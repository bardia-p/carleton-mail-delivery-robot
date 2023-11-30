#!/bin/bash

input_directory="./diagrams/plantuml_files"

plantuml="./tools/plantuml.jar"

if [ $(which dot 2>/dev/null) ]; then
  echo "Found GraphViz"
else
  echo "Did not find GraphViz"
  sudo apt install graphviz
fi

for file in "$input_directory"/*.plantuml; do
    if [ -f "$file" ]; then
      java -jar "$plantuml" "$file" -o "../"
      if [ $? != 0 ]; then
          exit 1
      fi
      echo "Generated a diagram."
    fi
done

echo "Successfully Generated the Diagrams"