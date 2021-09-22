include(bin2h.cmake)
bin2h(SOURCE_FILE "${SOURCE_FILE}"
        HEADER_FILE "${OUTPUT_FILE}"
        VARIABLE_NAME "${VARIABLE_NAME}")
