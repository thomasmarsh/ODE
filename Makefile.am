LIBTOOL_DEPS = @LIBTOOL_DEPS@
libtool: $(LIBTOOL_DEPS)
	$(SHELL) ./config.status --recheck

if ENABLE_DEMOS
SUBDIRS = include drawstuff ode 
else
SUBDIRS = include ode
endif

bin_SCRIPTS = ode-config

# Utility rule for making a release
release: dist dist-bz2 dist-zip
	@echo Created release packages for ${PACKAGE}-${VERSION}.

EXTRA_DIST = autogen.sh build contrib OPCODE GIMPACT \
        CHANGELOG.txt INSTALL.txt README.txt LICENSE.TXT

