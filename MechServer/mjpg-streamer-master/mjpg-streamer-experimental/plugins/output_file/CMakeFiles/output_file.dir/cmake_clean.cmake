FILE(REMOVE_RECURSE
  "CMakeFiles/output_file.dir/output_file.c.o"
  "output_file.pdb"
  "output_file.so"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang C)
  INCLUDE(CMakeFiles/output_file.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
