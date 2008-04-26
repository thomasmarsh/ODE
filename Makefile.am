LIBTOOL_DEPS = @LIBTOOL_DEPS@
libtool: $(LIBTOOL_DEPS)
	$(SHELL) ./config.status --recheck

SUBDIRS =

if ENABLE_OU
SUBDIRS += ou
endif

if ENABLE_DEMOS
SUBDIRS += include drawstuff GIMPACT OPCODE ode
else
SUBDIRS += include GIMPACT OPCODE ode
endif

bin_SCRIPTS = ode-config

# Utility rule for making a release
release: dist dist-gzip dist-bz2 dist-zip
	@echo Created release packages for ${PACKAGE}-${VERSION}.

EXTRA_DIST = autogen.sh build \
        CHANGELOG.txt INSTALL.txt README.txt LICENSE.TXT

