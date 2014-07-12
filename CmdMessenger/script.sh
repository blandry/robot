#!/bin/bash

for j in $(ls -1 _*); do
  string=${j#"_"}
  echo "Arquivo: $j, nome sem '_': $string"
  grep -rl $j ./ | xargs sed -i "s/$j/$string/g"

  mv $j $string
done

