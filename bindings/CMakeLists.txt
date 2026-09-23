# Fragment included by the root CMakeLists.txt (NAS_BUILD_BINDINGS) — no project()/find_package()
# of its own; CMAKE_POSITION_INDEPENDENT_CODE is set globally at the root now (the `nas` target is
# built once for the whole project, so there is no more "set it before add_subdirectory so it
# propagates" — the PIC bricolage this file used to need).

find_package(Python 3.8 COMPONENTS Interpreter Development.Module REQUIRED)

# nanobind ships as a pip package (its C++ headers + a small CMake package, not a Python module to
# import at runtime) — find it via the interpreter that's active when CMake runs, same pattern
# nanobind's own docs use.
execute_process(
  COMMAND "${Python_EXECUTABLE}" -m nanobind --cmake_dir
  OUTPUT_STRIP_TRAILING_WHITESPACE OUTPUT_VARIABLE NB_DIR)
list(APPEND CMAKE_PREFIX_PATH "${NB_DIR}")
find_package(nanobind CONFIG REQUIRED)

nanobind_add_module(nas_bindings src/module.cpp)
target_link_libraries(nas_bindings PRIVATE nas)
install(TARGETS nas_bindings LIBRARY DESTINATION .)

if(NAS_BUILD_TESTS)
  add_test(NAME nas_bindings_tests
      COMMAND "${Python_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/tests/test_bindings.py" "$<TARGET_FILE_DIR:nas_bindings>")
  add_test(NAME nas_bindings_memory
      COMMAND "${Python_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/tests/test_memory.py" "$<TARGET_FILE_DIR:nas_bindings>")
endif()
