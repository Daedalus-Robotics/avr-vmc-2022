file(REMOVE_RECURSE
  "libpaho-mqttpp3.pdb"
  "libpaho-mqttpp3.a"
)

# Per-language clean rules from dependency scanning.
foreach(lang CXX)
  include(CMakeFiles/paho-mqttpp3-static.dir/cmake_clean_${lang}.cmake OPTIONAL)
endforeach()
