
# determine source and header files
file(GLOB_RECURSE GUACAMOLE_PHYSICS_SRC RELATIVE ${CMAKE_CURRENT_SOURCE_DIR}
    src/*.cpp
    include/*.h
    include/*.hpp
)

SET(GUACAMOLE_PHYSICS_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include)

LINK_DIRECTORIES(${LIB_PATHS} ${OVR_LIBRARY_DIRS})

INCLUDE_DIRECTORIES( ${INCLUDE_PATHS}
                     ${GUACAMOLE_SOURCE_DIR}
                     ${OVR_INCLUDE_DIRS}
                     ${GUACAMOLE_PHYSICS_SOURCE_DIR}
)

ADD_LIBRARY( guacamole-physics SHARED
    ${GUACAMOLE_PHYSICS_SRC}
)

ADD_DEPENDENCIES(guacamole-physics guacamole)

IF (MSVC)
	set_target_properties(guacamole-physics PROPERTIES COMPILE_FLAGS "-D GUA_PHYSICS_LIBRARY")
ENDIF (MSVC)



TARGET_LINK_LIBRARIES( guacamole-physics debug ${LIBS} optimized ${LIBS})


