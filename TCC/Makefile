LATEX = xelatex -interaction=nonstopmode
BIB = bibtex
LATEXMK = latexmk -xelatex -interaction=nonstopmode

FONTES_TEX = main.tex $(wildcard capitulos/*.tex) $(wildcard partes/*.tex) $(wildcard pacotes/*.tex)
FIGURAS = $(wildcard figuras/*) $(wildcard configs/figuras/*)

all: main.pdf

main.pdf: $(FONTES_TEX) referencias.bib $(FIGURAS)
	@if command -v latexmk >/dev/null 2>&1; then \
		$(LATEXMK) main.tex; \
	else \
		$(LATEX) main.tex && \
		$(BIB) main.aux && \
		$(LATEX) main.tex && \
		$(LATEX) main.tex; \
	fi

clean:
	rm -f *.aux *.bbl *.blg *.bcf *.run.xml *.log *.toc *.out *.lof *.lot *.fls *.xdv *.fdb_latexmk "main.synctex(busy)"

cleanall: clean
	rm -f main.pdf

.PHONY: all clean cleanall

