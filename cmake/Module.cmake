# Module — sushiengine_add_module(), the one place in the build that grants a module a
# public include directory. Everything a module can see is therefore something it
# declared, and every declaration is checked against the tier order in EngineLayers
# before it is linked, so an illegal edge fails the configure instead of compiling.

# Pulled in here so a caller only needs include(Module); the guard keeps a repeated
# include harmless and still re-reads the manifest in a scope that cannot see it.
if(NOT DEFINED SUSHIENGINE_LAYER_ORDER)
    include(EngineLayers)
endif()

# sushiengine_module_edge_forbidden(<consumer> <dependency> <output_variable>)
#   Sets <output_variable> to TRUE when the exact pair is listed in
#   SUSHIENGINE_FORBIDDEN_EDGES, which no tier arrangement may make legal.
function(sushiengine_module_edge_forbidden consumer dependency output_variable)
    set(${output_variable} FALSE PARENT_SCOPE)

    list(LENGTH SUSHIENGINE_FORBIDDEN_EDGES entry_count)
    math(EXPR last_pair "${entry_count} - 2")
    set(index 0)
    while(index LESS_EQUAL last_pair)
        math(EXPR dependency_slot "${index} + 1")
        list(GET SUSHIENGINE_FORBIDDEN_EDGES ${index} listed_consumer)
        list(GET SUSHIENGINE_FORBIDDEN_EDGES ${dependency_slot} listed_dependency)
        if("${listed_consumer}" STREQUAL "${consumer}"
                AND "${listed_dependency}" STREQUAL "${dependency}")
            set(${output_variable} TRUE PARENT_SCOPE)
            return()
        endif()
        math(EXPR index "${index} + 2")
    endwhile()
endfunction()

# sushiengine_module_edge_exception(<consumer> <dependency> <date_variable> <reason_variable>)
#   Sets both variables from the matching SUSHIENGINE_LAYER_EXCEPTIONS entry, or to the
#   empty string when the pair carries no exception; a non-empty date is the match.
function(sushiengine_module_edge_exception consumer dependency date_variable reason_variable)
    set(${date_variable} "" PARENT_SCOPE)
    set(${reason_variable} "" PARENT_SCOPE)

    list(LENGTH SUSHIENGINE_LAYER_EXCEPTIONS entry_count)
    math(EXPR last_entry "${entry_count} - 4")
    set(index 0)
    while(index LESS_EQUAL last_entry)
        math(EXPR dependency_slot "${index} + 1")
        list(GET SUSHIENGINE_LAYER_EXCEPTIONS ${index} listed_consumer)
        list(GET SUSHIENGINE_LAYER_EXCEPTIONS ${dependency_slot} listed_dependency)
        if("${listed_consumer}" STREQUAL "${consumer}"
                AND "${listed_dependency}" STREQUAL "${dependency}")
            math(EXPR date_slot "${index} + 2")
            math(EXPR reason_slot "${index} + 3")
            list(GET SUSHIENGINE_LAYER_EXCEPTIONS ${date_slot} listed_date)
            list(GET SUSHIENGINE_LAYER_EXCEPTIONS ${reason_slot} listed_reason)
            set(${date_variable} "${listed_date}" PARENT_SCOPE)
            set(${reason_variable} "${listed_reason}" PARENT_SCOPE)
            return()
        endif()
        math(EXPR index "${index} + 4")
    endwhile()
endfunction()

# sushiengine_check_module_edge(<consumer> <dependency>)
#   Fails the configure when <consumer> may not depend on <dependency>: an unmanifested
#   module, a forbidden pair, or an upward edge carrying no dated exception. An excepted
#   upward edge is reported so the debt shows up in every configure log.
function(sushiengine_check_module_edge consumer dependency)
    sushiengine_module_layer("${dependency}" dependency_layer)
    if("${dependency_layer}" STREQUAL "")
        message(FATAL_ERROR
            "sushiengine_add_module(${consumer}): depends on '${dependency}', which is not "
            "in SUSHIENGINE_MODULE_LAYERS (cmake/EngineLayers.cmake).")
    endif()

    sushiengine_module_edge_forbidden("${consumer}" "${dependency}" forbidden)
    if(forbidden)
        message(FATAL_ERROR
            "sushiengine_add_module(${consumer}): ${consumer} -> ${dependency} is listed in "
            "SUSHIENGINE_FORBIDDEN_EDGES and is refused whatever the tier order says.")
    endif()

    sushiengine_module_layer("${consumer}" consumer_layer)
    sushiengine_layer_index("${consumer_layer}" consumer_index)
    sushiengine_layer_index("${dependency_layer}" dependency_index)
    if(consumer_index EQUAL -1 OR dependency_index EQUAL -1)
        message(FATAL_ERROR
            "sushiengine_add_module(${consumer}): ${consumer} (${consumer_layer}) or "
            "${dependency} (${dependency_layer}) is manifested in a layer that is not in "
            "SUSHIENGINE_LAYER_ORDER (cmake/EngineLayers.cmake).")
    endif()

    if(dependency_index GREATER consumer_index)
        sushiengine_module_edge_exception("${consumer}" "${dependency}"
            exception_date exception_reason)
        if("${exception_date}" STREQUAL "")
            message(FATAL_ERROR
                "sushiengine_add_module(${consumer}): upward dependency ${consumer} "
                "(${consumer_layer}) -> ${dependency} (${dependency_layer}). A module may "
                "only depend on its own layer or a lower one; move the shared code down, or "
                "add a dated entry to SUSHIENGINE_LAYER_EXCEPTIONS.")
        else()
            message(STATUS
                "sushiengine: layer exception ${consumer} -> ${dependency} "
                "(${exception_date}): ${exception_reason}")
        endif()
    endif()
endfunction()

# sushiengine_add_module(NAME <module> LAYER <layer> [TYPE STATIC|INTERFACE|OBJECT]
#                        [SOURCES <file>...] [PUBLIC_DEPENDS <module>...]
#                        [PRIVATE_DEPENDS <module>...] [EXTERNAL_PUBLIC <target>...]
#                        [EXTERNAL_PRIVATE <target>...])
#   Declares one engine module as the target sushiengine_<module>, carrying the module
#   directory's own include/ as its public header root and source/ as a private one,
#   linking the engine modules whose edges EngineLayers permits and the raw CMake targets
#   given as external. TYPE defaults to INTERFACE without SOURCES and STATIC with them,
#   and SOURCES are relative to the module directory.
function(sushiengine_add_module)
    cmake_parse_arguments(ARG
        ""
        "NAME;LAYER;TYPE"
        "SOURCES;PUBLIC_DEPENDS;PRIVATE_DEPENDS;EXTERNAL_PUBLIC;EXTERNAL_PRIVATE"
        ${ARGN})

    if("${ARG_NAME}" STREQUAL "")
        message(FATAL_ERROR "sushiengine_add_module(): NAME is required.")
    endif()
    if(NOT "${ARG_UNPARSED_ARGUMENTS}" STREQUAL "")
        message(FATAL_ERROR
            "sushiengine_add_module(${ARG_NAME}): unexpected argument(s) "
            "'${ARG_UNPARSED_ARGUMENTS}'.")
    endif()
    if(NOT "${ARG_KEYWORDS_MISSING_VALUES}" STREQUAL "")
        message(FATAL_ERROR
            "sushiengine_add_module(${ARG_NAME}): keyword(s) given with no value "
            "'${ARG_KEYWORDS_MISSING_VALUES}'.")
    endif()
    if("${ARG_LAYER}" STREQUAL "")
        message(FATAL_ERROR "sushiengine_add_module(${ARG_NAME}): LAYER is required.")
    endif()

    sushiengine_module_layer("${ARG_NAME}" manifest_layer)
    if("${manifest_layer}" STREQUAL "")
        message(FATAL_ERROR
            "sushiengine_add_module(${ARG_NAME}): '${ARG_NAME}' is not in "
            "SUSHIENGINE_MODULE_LAYERS (cmake/EngineLayers.cmake). Add it there first.")
    endif()
    if(NOT "${ARG_LAYER}" STREQUAL "${manifest_layer}")
        message(FATAL_ERROR
            "sushiengine_add_module(${ARG_NAME}): declares LAYER '${ARG_LAYER}' but the "
            "manifest places the module in '${manifest_layer}'.")
    endif()

    if(NOT "${ARG_TYPE}" STREQUAL "")
        set(module_type "${ARG_TYPE}")
    elseif(ARG_SOURCES)
        set(module_type STATIC)
    else()
        set(module_type INTERFACE)
    endif()
    if(NOT module_type MATCHES "^(STATIC|INTERFACE|OBJECT)$")
        message(FATAL_ERROR
            "sushiengine_add_module(${ARG_NAME}): TYPE '${module_type}' is none of STATIC, "
            "INTERFACE, OBJECT.")
    endif()
    if(module_type STREQUAL "INTERFACE" AND ARG_SOURCES)
        message(FATAL_ERROR
            "sushiengine_add_module(${ARG_NAME}): an INTERFACE module compiles nothing, so "
            "SOURCES cannot be given.")
    endif()

    set(target "sushiengine_${ARG_NAME}")
    set(module_sources "")
    foreach(source IN LISTS ARG_SOURCES)
        list(APPEND module_sources "${CMAKE_CURRENT_SOURCE_DIR}/${source}")
    endforeach()

    if(module_type STREQUAL "INTERFACE")
        add_library(${target} INTERFACE)
        target_include_directories(${target}
            INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}/include")
    else()
        add_library(${target} ${module_type} ${module_sources})
        target_include_directories(${target}
            PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include"
            PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/source")
    endif()

    foreach(dependency IN LISTS ARG_PUBLIC_DEPENDS ARG_PRIVATE_DEPENDS)
        sushiengine_check_module_edge("${ARG_NAME}" "${dependency}")
    endforeach()

    set(public_links "")
    foreach(dependency IN LISTS ARG_PUBLIC_DEPENDS)
        list(APPEND public_links "sushiengine_${dependency}")
    endforeach()
    list(APPEND public_links ${ARG_EXTERNAL_PUBLIC})

    set(private_links "")
    foreach(dependency IN LISTS ARG_PRIVATE_DEPENDS)
        list(APPEND private_links "sushiengine_${dependency}")
    endforeach()
    list(APPEND private_links ${ARG_EXTERNAL_PRIVATE})

    if(module_type STREQUAL "INTERFACE")
        # A header-only module has no implementation to keep to itself, so what it asked to
        # keep private is still part of what a consumer has to link.
        if(public_links OR private_links)
            target_link_libraries(${target} INTERFACE ${public_links} ${private_links})
        endif()
    else()
        if(public_links)
            target_link_libraries(${target} PUBLIC ${public_links})
        endif()
        if(private_links)
            target_link_libraries(${target} PRIVATE ${private_links})
        endif()
        set_target_properties(${target} PROPERTIES
            CXX_STANDARD 17
            CXX_STANDARD_REQUIRED ON)
    endif()
endfunction()
