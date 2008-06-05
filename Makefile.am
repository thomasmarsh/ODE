LIBTOOL_DEPS = @LIBTOOL_DEPS@
libtool: $(LIBTOOL_DEPS)
	$(SHELL) ./config.status --recheck

if ENABLE_OU
OU_DIR = ou
endif

if ENABLE_DEMOS
DRAWSTUFF_DIR = drawstuff
endif

if GIMPACT
GIMPACT_DIR = GIMPACT
endif

if OPCODE
OPCODE_DIR = OPCODE
endif

SUBDIRS = include \
          $(DRAWSTUFF_DIR) \
          $(GIMPACT_DIR) \
          $(OPCODE_DIR) \
          $(OU_DIR) \
          ode \
          tests

bin_SCRIPTS = ode-config

# Utility rule for making a release
release: dist dist-gzip dist-bzip2 dist-zip
	@echo Created release packages for ${PACKAGE}-${VERSION}.

EXTRA_DIST = autogen.sh build tools \
        CHANGELOG.txt INSTALL.txt README.txt LICENSE.TXT

