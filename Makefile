.PHONY: all doc doc-man doc-latex docclean

doc:
	"$(MAKE)" -BC doc/doxygen

doc-man:
	"$(MAKE)" -BC doc/doxygen man

latex:
	"$(MAKE)" -BC doc/doxygen latex

docclean:
	"$(MAKE)" -BC doc/doxygen clean

