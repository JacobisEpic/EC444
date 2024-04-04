# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "/Users/jacob/esp/esp-idf/components/bootloader/subproject"
  "/Users/jacob/Documents/EC444/Chin-Jacob/skills/cluster-2/15/code/blink/build/bootloader"
  "/Users/jacob/Documents/EC444/Chin-Jacob/skills/cluster-2/15/code/blink/build/bootloader-prefix"
  "/Users/jacob/Documents/EC444/Chin-Jacob/skills/cluster-2/15/code/blink/build/bootloader-prefix/tmp"
  "/Users/jacob/Documents/EC444/Chin-Jacob/skills/cluster-2/15/code/blink/build/bootloader-prefix/src/bootloader-stamp"
  "/Users/jacob/Documents/EC444/Chin-Jacob/skills/cluster-2/15/code/blink/build/bootloader-prefix/src"
  "/Users/jacob/Documents/EC444/Chin-Jacob/skills/cluster-2/15/code/blink/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/Users/jacob/Documents/EC444/Chin-Jacob/skills/cluster-2/15/code/blink/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/Users/jacob/Documents/EC444/Chin-Jacob/skills/cluster-2/15/code/blink/build/bootloader-prefix/src/bootloader-stamp${cfgdir}") # cfgdir has leading slash
endif()
