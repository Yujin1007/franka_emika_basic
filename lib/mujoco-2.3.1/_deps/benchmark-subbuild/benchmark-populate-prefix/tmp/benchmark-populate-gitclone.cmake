
if(NOT "/home/kist-robot2/mujoco-2.3.1_source/mujoco-2.3.1/_deps/benchmark-subbuild/benchmark-populate-prefix/src/benchmark-populate-stamp/benchmark-populate-gitinfo.txt" IS_NEWER_THAN "/home/kist-robot2/mujoco-2.3.1_source/mujoco-2.3.1/_deps/benchmark-subbuild/benchmark-populate-prefix/src/benchmark-populate-stamp/benchmark-populate-gitclone-lastrun.txt")
  message(STATUS "Avoiding repeated git clone, stamp file is up to date: '/home/kist-robot2/mujoco-2.3.1_source/mujoco-2.3.1/_deps/benchmark-subbuild/benchmark-populate-prefix/src/benchmark-populate-stamp/benchmark-populate-gitclone-lastrun.txt'")
  return()
endif()

execute_process(
  COMMAND ${CMAKE_COMMAND} -E remove_directory "/home/kist-robot2/mujoco-2.3.1_source/mujoco-2.3.1/_deps/benchmark-src"
  RESULT_VARIABLE error_code
  )
if(error_code)
  message(FATAL_ERROR "Failed to remove directory: '/home/kist-robot2/mujoco-2.3.1_source/mujoco-2.3.1/_deps/benchmark-src'")
endif()

# try the clone 3 times in case there is an odd git clone issue
set(error_code 1)
set(number_of_tries 0)
while(error_code AND number_of_tries LESS 3)
  execute_process(
    COMMAND "/usr/bin/git"  clone --no-checkout "https://github.com/google/benchmark.git" "benchmark-src"
    WORKING_DIRECTORY "/home/kist-robot2/mujoco-2.3.1_source/mujoco-2.3.1/_deps"
    RESULT_VARIABLE error_code
    )
  math(EXPR number_of_tries "${number_of_tries} + 1")
endwhile()
if(number_of_tries GREATER 1)
  message(STATUS "Had to git clone more than once:
          ${number_of_tries} times.")
endif()
if(error_code)
  message(FATAL_ERROR "Failed to clone repository: 'https://github.com/google/benchmark.git'")
endif()

execute_process(
  COMMAND "/usr/bin/git"  checkout d572f4777349d43653b21d6c2fc63020ab326db2 --
  WORKING_DIRECTORY "/home/kist-robot2/mujoco-2.3.1_source/mujoco-2.3.1/_deps/benchmark-src"
  RESULT_VARIABLE error_code
  )
if(error_code)
  message(FATAL_ERROR "Failed to checkout tag: 'd572f4777349d43653b21d6c2fc63020ab326db2'")
endif()

set(init_submodules TRUE)
if(init_submodules)
  execute_process(
    COMMAND "/usr/bin/git"  submodule update --recursive --init 
    WORKING_DIRECTORY "/home/kist-robot2/mujoco-2.3.1_source/mujoco-2.3.1/_deps/benchmark-src"
    RESULT_VARIABLE error_code
    )
endif()
if(error_code)
  message(FATAL_ERROR "Failed to update submodules in: '/home/kist-robot2/mujoco-2.3.1_source/mujoco-2.3.1/_deps/benchmark-src'")
endif()

# Complete success, update the script-last-run stamp file:
#
execute_process(
  COMMAND ${CMAKE_COMMAND} -E copy
    "/home/kist-robot2/mujoco-2.3.1_source/mujoco-2.3.1/_deps/benchmark-subbuild/benchmark-populate-prefix/src/benchmark-populate-stamp/benchmark-populate-gitinfo.txt"
    "/home/kist-robot2/mujoco-2.3.1_source/mujoco-2.3.1/_deps/benchmark-subbuild/benchmark-populate-prefix/src/benchmark-populate-stamp/benchmark-populate-gitclone-lastrun.txt"
  RESULT_VARIABLE error_code
  )
if(error_code)
  message(FATAL_ERROR "Failed to copy script-last-run stamp file: '/home/kist-robot2/mujoco-2.3.1_source/mujoco-2.3.1/_deps/benchmark-subbuild/benchmark-populate-prefix/src/benchmark-populate-stamp/benchmark-populate-gitclone-lastrun.txt'")
endif()

