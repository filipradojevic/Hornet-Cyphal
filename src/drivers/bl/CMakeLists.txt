idf_component_register(
    SRCS
        "bl.c"
    INCLUDE_DIRS
        "."
	REQUIRES
		driver
		app_update
		mav
)

