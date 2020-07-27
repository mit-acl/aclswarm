#!/bin/bash

exts="aux bbl blg brf idx ilg ind lof log lol lot out toc synctex.gz fls fdb_latexmk"

for ext in $exts; do
    find . -path ./figures -prune -o -type f -name "*.$ext" -exec rm {} +
done

# autocmd BufWritePost,FileWritePost *.tex :silent! execute "!xelatex report.tex >/dev/null 2>&1 && xdg-open report.pdf" | redraw!
