#!/usr/bin/env python3
"""
gen_ulog_cyphal_full.py
Robust DSDL -> ULog (.h/.c) generator for UAVCAN/Cyphal messages.

Features:
 - Indexes all .dsdl files under provided root
 - Resolves fully-qualified custom types recursively
 - Inlines Scalar-like types that have single 'value' field
 - Flattens compound custom types (parent_child naming)
 - Supports primitive type mapping
 - Handles bitfields like uint5 / int3 and packs contiguous bitfields into minimal backing integer
 - Supports arrays type[N] and type[<=N] (uses N)
 - Optional addition of timestamp as first field (--add-timestamp)
 - Preserves directory structure in output
"""
import os
import argparse
import re
from jinja2 import Template

# ---------- Templates ----------
HEADER_TEMPLATE = r"""/**
 * @file    ulog_{{ name }}.h
 * @brief   Auto-generated from DSDL.
 */

#ifndef ULOG_{{ name_caps }}_H
#define ULOG_{{ name_caps }}_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "ulog.h"

{% if packed %}#ifdef __GNUC__
#define ULOG_PACKED __attribute__((packed))
#else
#define ULOG_PACKED
#endif
{% else %}
#define ULOG_PACKED
{% endif %}

typedef struct ULOG_PACKED ulog_{{ name }}_t {
    {% if add_timestamp %}uint64_t timestamp; /* Timestamp [us] */{% endif %}
    {% for f in fields %}
    {{ f.decl }}{% if f.comment %} /* {{ f.comment }} */{% endif %}
    {% endfor %}
} ulog_{{ name }}_t;

int32_t ulog_format_{{ name }}(ulog_t *log);
int32_t ulog_subscribe_{{ name }}(ulog_t *log, uint8_t multi_id, uint16_t *id);
int32_t ulog_write_{{ name }}(ulog_t *log, ulog_msg_id_t id, const ulog_{{ name }}_t *src);

#ifdef __cplusplus
}
#endif

#endif
"""

SOURCE_TEMPLATE = r"""/**
 * @file    ulog_{{ name }}.c
 * @brief   Auto-generated from DSDL.
 */

#include "ulog_{{ name }}.h"
#include "ulog_def.h"

const char ulog_{{ name }}_format[] =
"{{ short_name }}:"
{% if add_timestamp %}"uint64_t timestamp;"{% endif %}
{% for f in fields -%}
"{{ f.format_token }}{% if f.array > 0 %}[{{ f.array }}]{% endif %} {{ f.basename }};"
{% endfor %}
;

const char ulog_{{ name }}_name[] = "{{ short_name }}";

int32_t ulog_format_{{ name }}(ulog_t *log)
{
    ulog_message_header_t header = {
        .msg_size = sizeof(ulog_{{ name }}_format) - 1,
        .msg_type = ULOG_DEF_MSG_TYPE_FORMAT_DEFINITION,
    };

    if(log->state == ULOG_STATE_HEADER_CREATED)
        log->state = ULOG_STATE_DEFINITIONS_CREATED;

    if(log->state != ULOG_STATE_DEFINITIONS_CREATED)
        return 1;

    if (lfs_file_write(log->lfs, log->file, &header, sizeof(header)) < 0)
        return 1;

    if (lfs_file_write(log->lfs, log->file, ulog_{{ name }}_format, sizeof(ulog_{{ name }}_format) - 1) < 0)
        return 1;

    return 0;
}

int32_t ulog_subscribe_{{ name }}(ulog_t *log, uint8_t multi_id, uint16_t *id)
{
    ulog_message_header_t header = {
        .msg_size = sizeof(uint8_t) + sizeof(uint16_t) + sizeof(ulog_{{ name }}_name) - 1,
        .msg_type = ULOG_DEF_MSG_TYPE_SUBSCRIPTION,
    };

    if(log->state == ULOG_STATE_DEFINITIONS_CREATED)
        log->state = ULOG_STATE_WRITING_DATA;

    if(log->state != ULOG_STATE_WRITING_DATA)
        return 1;

    if (lfs_file_write(log->lfs, log->file, &header, sizeof(header)) < 0)
        return 1;

    if (lfs_file_write(log->lfs, log->file, &multi_id, sizeof(multi_id)) < 0)
        return 1;

    if (lfs_file_write(log->lfs, log->file, &log->msg_cnt, sizeof(log->msg_cnt)) < 0)
        return 1;

    if (lfs_file_write(log->lfs, log->file, ulog_{{ name }}_name, sizeof(ulog_{{ name }}_name) - 1) < 0)
        return 1;

    *id = log->msg_cnt++;

    return 0;
}

int32_t ulog_write_{{ name }}(ulog_t *log, ulog_msg_id_t id, const ulog_{{ name }}_t *src)
{
    ulog_message_header_t header = {
        .msg_size = sizeof(uint16_t)
                    {% if add_timestamp %}+ sizeof(src->timestamp){% endif %}
                    {% for f in write_size_fields %}
                    + {{ f.size_expr }}
                    {% endfor %},
        .msg_type = ULOG_DEF_MSG_TYPE_LOGGED_DATA,
    };

    if(log->state != ULOG_STATE_WRITING_DATA || id >= log->msg_cnt)
        return 1;

    if (lfs_file_write(log->lfs, log->file, &header, sizeof(header)) < 0)
        return 1;

    if (lfs_file_write(log->lfs, log->file, &id, sizeof(id)) < 0)
        return 1;

    {% if add_timestamp %}
    if (lfs_file_write(log->lfs, log->file, &src->timestamp, sizeof(src->timestamp)) < 0)
        return 1;
    {% endif %}

    {% for f in write_fields %}
    {{ f.write_stmt }}
    {% endfor %}

    return 0;
}
"""

# ---------- Primitive mapping ----------
# map common DSDL tokens and synonyms (lowercase) to (C type, format token, size in bytes)
PRIMITIVES = {
    "bool": ("bool", "bool", 1),
    "bit": ("bool", "bool", 1),
    "int8": ("int8_t", "int8_t", 1),
    "integer8": ("int8_t", "int8_t", 1),
    "int16": ("int16_t", "int16_t", 2),
    "integer16": ("int16_t", "int16_t", 2),
    "int32": ("int32_t", "int32_t", 4),
    "integer32": ("int32_t", "int32_t", 4),
    "int64": ("int64_t", "int64_t", 8),
    "integer64": ("int64_t", "int64_t", 8),
    "uint8": ("uint8_t", "uint8_t", 1),
    "natural8": ("uint8_t", "uint8_t", 1),
    "uint16": ("uint16_t", "uint16_t", 2),
    "natural16": ("uint16_t", "uint16_t", 2),
    "uint32": ("uint32_t", "uint32_t", 4),
    "natural32": ("uint32_t", "uint32_t", 4),
    "uint64": ("uint64_t", "uint64_t", 8),
    "natural64": ("uint64_t", "uint64_t", 8),
    "float16": ("uint16_t", "uint16_t", 2),  # store raw
    "real16": ("uint16_t", "uint16_t", 2),
    "float32": ("float", "float", 4),
    "real32": ("float", "float", 4),
    "float64": ("double", "double", 8),
    "real64": ("double", "double", 8),
    "string": ("char", "char", 1),
    "unstructured": ("uint8_t", "uint8_t", 1),
}


# ---------- DSDL indexing ----------
class DsdlIndex:
    def __init__(self, root):
        self.root = os.path.abspath(root)
        self.index = {}  # canonical -> filepath
        self.build_index()

    def build_index(self):
        for dp, _, fns in os.walk(self.root):
            for fn in fns:
                if not fn.endswith(".dsdl"):
                    continue
                full = os.path.join(dp, fn)
                rel = os.path.relpath(full, self.root)
                parts = rel.split(os.sep)
                last = parts[-1].replace(".dsdl", "")
                a = last.split(".")
                if len(a) >= 3 and a[-1].isdigit() and a[-2].isdigit():
                    basename = ".".join(a[:-2])
                else:
                    basename = last.rsplit(".", 2)[0] if "." in last else last
                ns = ".".join(p for p in parts[:-1] if p)
                if ns:
                    canonical = f"{ns}.{basename}"
                else:
                    canonical = basename
                self.index[canonical] = full
                self.index[canonical.lower()] = full

    def lookup(self, tok):
        tok = tok.strip()
        # strip trailing version if present
        if tok.endswith(".dsdl"):
            tok = tok[:-5]
        # remove trailing .1.0 or .0.1 if present
        parts = tok.split(".")
        if len(parts) >= 3 and parts[-1].isdigit() and parts[-2].isdigit():
            tok_short = ".".join(parts[:-2])
        else:
            tok_short = tok
        if tok in self.index:
            return self.index[tok]
        if tok_short in self.index:
            return self.index[tok_short]
        if tok.lower() in self.index:
            return self.index[tok.lower()]
        if tok_short.lower() in self.index:
            return self.index[tok_short.lower()]
        # try suffix match
        last = tok.split(".")[-1].lower()
        for k, v in self.index.items():
            if k.lower().endswith("." + last) or k.lower() == last:
                return v
        return None

# ---------- Parsing raw DSDL lines ----------
def parse_dsdl_raw(path):
    lines = []
    with open(path, "r", encoding="utf-8") as f:
        for raw in f:
            s = raw.strip()
            if not s:
                continue
            if s.startswith("#"):
                continue
            if s.startswith("@"):
                continue
            # strip inline comments //
            if "//" in s:
                s = s.split("//", 1)[0].strip()
            # skip constants with '='
            if "=" in s:
                continue
            parts = s.split()
            if len(parts) < 2:
                continue
            lines.append(s)
    return lines

# ---------- Resolve a field (possibly custom) ----------
def parse_array_size(token):
    # token like "float32[3]" or "Type[<=3]"
    if "[" in token and "]" in token:
        base, rest = token.split("[", 1)
        arr = rest.split("]", 1)[0]
        if "<=" in arr:
            m = re.search(r"\d+", arr)
            if m:
                return base, int(m.group(0))
            return base, 0
        try:
            return base, int(arr)
        except:
            return base, 0
    return token, 0

def map_primitive(tok):
    key = tok.lower()
    return PRIMITIVES.get(key)

# resolved_cache to avoid repeated resolution cycles
def resolve_type(token, field_name, dsdl_index, resolved_cache):
    """
    Returns list of resolved field dicts:
      { name, ctype, format_token, array, comment, bitsize(optional) }
    """
    base, arr = parse_array_size(token)
    # handle bit sizes like uint5, int3, bit5 etc.
    m = re.match(r'^(u?int|integer|natural|uint|int|bit|boolean)(\d{1,2})$', base, re.IGNORECASE)
    if m:
        # treat as integer with explicit bit width
        prefix = m.group(1).lower()
        width = int(m.group(2))
        # map prefix to signedness
        signed = prefix in ("int", "integer")
        # produce bitfield descriptor
        return [{"name": field_name, "ctype": None, "format_token": ("uint" if not signed else "int") + str(width),
                 "array": arr, "comment": "", "bits": width, "signed": signed}]
    # map primitives
    prim = map_primitive(base)
    if prim:
        ctype, fmt, size = prim
        return [{"name": field_name, "ctype": ctype, "format_token": fmt, "array": arr, "comment": "", "bytes": size}]
    # custom type: find dsdl
    dsdl_path = dsdl_index.lookup(base)
    if not dsdl_path:
        # unknown type -> skip silently
        return []
    cache_key = (dsdl_path, field_name, arr)
    if cache_key in resolved_cache:
        return resolved_cache[cache_key]
    raw = parse_dsdl_raw(dsdl_path)
    child_fields = []
    for ln in raw:
        parts = ln.split()
        if len(parts) < 2:
            continue
        child_type = parts[0]
        child_name = parts[1]
        # recursively resolve
        resolved_children = resolve_type(child_type, child_name, dsdl_index, resolved_cache)
        # carry comments if present after name
        comment = " ".join(parts[2:]) if len(parts) > 2 else ""
        for rc in resolved_children:
            if comment:
                rc["comment"] = (rc.get("comment","") + " " + comment).strip()
            child_fields.append(rc)
    # if single child named 'value' -> inline it
    if len(child_fields) == 1 and child_fields[0]["name"].lower() in ("value", "v"):
        out = child_fields[0].copy()
        out["name"] = field_name
        # apply array from outer
        if arr and out.get("array", 0) == 0:
            out["array"] = arr
        resolved_cache[cache_key] = [out]
        return [out]
    # else flatten: prefix child names
    flattened = []
    for cf in child_fields:
        new = cf.copy()
        new["name"] = f"{field_name}_{cf['name']}"
        # if outer has array, apply to child if child not array
        if arr and new.get("array", 0) == 0:
            new["array"] = arr
        flattened.append(new)
    resolved_cache[cache_key] = flattened
    return flattened


def container_for_bits(nbits: int):
    """Return minimal C fixed-width integer type that can hold nbits."""
    if nbits <= 8:
        return "uint8_t", 1
    if nbits <= 16:
        return "uint16_t", 2
    if nbits <= 32:
        return "uint32_t", 4
    if nbits <= 64:
        return "uint64_t", 8
    raise ValueError(f"Bitfield too large: {nbits} bits")

def group_bitfields(fields):
    """
    Groups only TRUE bitfields (uint2, uint3, uint5, uint6, uint7...)
    Arrays are NEVER bitfields.
    Standard aligned types (8,16,32,64-bit) are NEVER bitfields.
    """
    grouped = []
    i = 0

    while i < len(fields):
        f = fields[i]

        # If field is array or aligned (byte boundary) → not a bitfield
        if f.get("array", 0) != 0 or f.get("bits", 0) in (8, 16, 32, 64):
            grouped.append(f)
            i += 1
            continue

        # Otherwise it is a bitfield → start a group
        if "bits" not in f:
            grouped.append(f)
            i += 1
            continue

        # Start bitfield pack
        bit_group = [f]
        total_bits = f["bits"]
        j = i + 1

        # Group until non-bitfield or array found
        while j < len(fields):
            nf = fields[j]

            # Stop if next is array or aligned type
            if nf.get("array", 0) != 0 or nf.get("bits", 0) in (8, 16, 32, 64):
                break

            if "bits" not in nf:
                break

            # ok, add it
            bit_group.append(nf)
            total_bits += nf["bits"]

            j += 1

        # pack into smallest container
        ctype, cbytes = container_for_bits(total_bits)
        grouped.append({
            "name": "_bitpack_" + fields[i]["name"],
            "ctype": ctype,
            "bytes": cbytes,
            "bitfields": bit_group
        })

        i = j

    return grouped


# utility: prepare final decl string and write statements
def prepare_final_fields(raw_fields, add_timestamp=False):
    """
    raw_fields: list of resolved dicts with keys: name, ctype or bits, array, comment, format_token, bytes
    Produces list of field descriptors for templates:
      - decl: C declaration string (with array)
      - basename: name without prefix for format string (we use last component)
      - format_token: token for format string
      - array: int
      - comment: str
      - write_stmt: C code to write this field into file
      - size_expr: expression used to compute size in header
    """
    # First group contiguous bitfields into bitfield decls
    grouped = group_bitfields(raw_fields)

    final = []
    write_fields = []
    size_fields = []
    for f in grouped:
        name = f["name"]
        arr = int(f.get("array", 0) or 0)
        comment = f.get("comment","")
        fmt = f.get("format_token", None)
        # bitfield case: has 'decl' already
        if "decl" in f:
            decl = f["decl"]
            # format token use underlying container type
            fmt_token = f.get("format_token", "uint16_t")
            basename = name
            final.append({"decl": decl, "basename": basename, "format_token": fmt_token, "array": 0, "comment": comment})
            # writing: write backing bytes as element (we need to write the whole underlying integer)
            # choose backing bytes by 'bytes' if present
            bytesz = f.get("bytes", 2)
            write_stmt = f'/* bitfield {name} */ if (lfs_file_write(log->lfs, log->file, &src->{name}, sizeof(src->{name})) < 0) return 1;'
            final[-1]["write_stmt"] = write_stmt
            size_fields.append({"size_expr": f"sizeof(src->{name})"})
            write_fields.append({"write_stmt": write_stmt})
            continue

        # non-bitfield: must have ctype and format_token
        ctype = f.get("ctype")
        if not ctype:
            # maybe format_token present (e.g., "uint5") — fallback skip
            continue
        decl = f"{ctype} {name}" + (f"[{arr}]" if arr > 0 else ";")
        if arr > 0:
            decl = f"{ctype} {name}[{arr}];"
            write_stmt = f'/* {comment} */ if (lfs_file_write(log->lfs, log->file, src->{name}, sizeof(src->{name})) < 0) return 1;'
            size_fields.append({"size_expr": f"sizeof(src->{name})"})
        else:
            write_stmt = f'/* {comment} */ if (lfs_file_write(log->lfs, log->file, &src->{name}, sizeof(src->{name})) < 0) return 1;'
            size_fields.append({"size_expr": f"sizeof(src->{name})"})
        basename = name.split("_")[-1]  # use last part for format string field name
        final.append({"decl": decl, "basename": basename, "format_token": fmt, "array": arr, "comment": comment})
        write_fields.append({"write_stmt": write_stmt})
    return final, write_fields, size_fields

# ---------- Main flow ----------
def main():
    parser = argparse.ArgumentParser(description="Generate ULog .c/.h from DSDL directory (Cyphal).")
    parser.add_argument("root", help="DSDL root directory (messages/cyphal)")
    parser.add_argument("-o", "--output", default="build/ulog", help="Output root")
    parser.add_argument("--add-timestamp", action="store_true", help="Add uint64_t timestamp as first field in structs")
    args = parser.parse_args()

    root = os.path.abspath(args.root)
    outroot = os.path.abspath(args.output)
    os.makedirs(outroot, exist_ok=True)

    dsdl_index = DsdlIndex(root)
    resolved_cache = {}

    # Walk and generate
    for dirpath, _, filenames in os.walk(root):
        for fn in filenames:
            if not fn.endswith(".dsdl"):
                continue
            full = os.path.join(dirpath, fn)
            # name + short
            rel = os.path.relpath(full, root)
            # build name like uavcan_si_unit_pressure_scalar
            parts = rel.split(os.sep)
            last = parts[-1].replace(".dsdl","")
            a = last.split(".")
            # Preserve version suffix for output naming where available (e.g., FooBar.0.1 -> foobar_0_1)
            if len(a) >= 3 and a[-1].isdigit() and a[-2].isdigit():
                basename = ".".join(a[:-2])
                ver_major, ver_minor = a[-2], a[-1]
                basename_with_ver = f"{basename}_{ver_major}_{ver_minor}"
            else:
                basename = last.rsplit(".",2)[0] if "." in last else last
                basename_with_ver = basename
            ns = parts[:-1]
            # Build output name including version suffix when present
            def _sanitize(tok: str) -> str:
                # Replace any non-alnum/underscore with underscore
                return re.sub(r"[^A-Za-z0-9_]", "_", tok)

            allparts = ns + [basename_with_ver]
            name = "_".join(_sanitize(p.lower()) for p in allparts if p)
            # Short name used in ULog format definition (string token)
            short = _sanitize(basename_with_ver.replace(".", "_").lower())
            out_dir = os.path.join(outroot, os.path.relpath(dirpath, root))
            os.makedirs(out_dir, exist_ok=True)

            # parse raw dsdl and resolve fields
            raw_lines = parse_dsdl_raw(full)
            resolved = []
            for ln in raw_lines:
                partsln = ln.split()
                if len(partsln) < 2:
                    continue
                tkn = partsln[0]
                fname = partsln[1]
                comment = " ".join(partsln[2:]) if len(partsln) > 2 else ""
                res = resolve_type(tkn, fname, dsdl_index, resolved_cache)
                # attach comments
                for r in res:
                    if comment:
                        r["comment"] = (r.get("comment","") + " " + comment).strip()
                    resolved.append(r)
            # prepare final fields (bitfield grouping + decl/write)
            final_fields, write_fields, size_fields = prepare_final_fields(resolved, add_timestamp=args.add_timestamp)

            # Determine if any packed attributes needed (if bitfields used)
            packed = any(":" in f.get("decl","") for f in final_fields)

            # Render templates
            header = Template(HEADER_TEMPLATE).render(name=name, name_caps=name.upper(), fields=final_fields, add_timestamp=args.add_timestamp, packed=packed)
            source = Template(SOURCE_TEMPLATE).render(name=name, short_name=short, fields=final_fields, add_timestamp=args.add_timestamp, write_fields=write_fields, write_size_fields=size_fields)

            hpath = os.path.join(out_dir, f"ulog_{name}.h")
            cpath = os.path.join(out_dir, f"ulog_{name}.c")
            with open(hpath, "w", encoding="utf-8") as fh:
                fh.write(header)
            with open(cpath, "w", encoding="utf-8") as fc:
                fc.write(source)

    print("Done: generated ULog .h/.c files under", outroot)

if __name__ == "__main__":
    main()