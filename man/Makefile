PROGB	=libmpu6050
PROGV	=1

INSTALL	=install
PANDOC	=pandoc
CMPRPRG	=gzip --force
CMPROPT	=--force
CMPREXT	=gz
MANDIR	=/usr/share/man
MANPAGE	=1
MANDEST =$(MANDIR)/man$(MANPAGE)
MANFILE	=$(PROGB).$(MANPAGE)
MANCMPR	=$(MANFILE).$(CMPREXT)

man:	$(PROGB).md
	-$(PANDOC) --standalone --to man $(PROGB).md -o $(MANFILE)
	-$(CMPRPRG) $(CMPROPT) $(MANFILE)

install:
	-sudo $(INSTALL) -D --mode 0644 $(MANCMPR) $(MANDEST)

uninstall:
	-sudo rm $(MANDEST)/$(MANCMPR)

clean:
	-rm -f $(MANFILE) $(MANCMPR)

remove: uninstall

all: man

.PHONY: all clean
