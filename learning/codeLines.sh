# read file extension
read -t 30 -p "Please input file name:"  file

# count code lines with exact file extension
codelines=$(find  ./ -name "$file" | xargs cat | grep -v ^$ | wc -l)
echo $codelines
