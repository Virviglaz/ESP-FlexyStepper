idf_component_register(SRCS "src/ESP_FlexyStepper.cpp"
	INCLUDE_DIRS "inc")

set_target_properties(${COMPONENT_LIB} PROPERTIES COMPILE_FLAGS -Wmissing-field-initializers)