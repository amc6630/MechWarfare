FILE(REMOVE_RECURSE
  "CMakeFiles/input_file.dir/input_file.c.o"
  "input_file.pdb"
  "input_file.so"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang C)
  INCLUDE(CMakeFiles/input_file.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
