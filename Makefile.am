LIBTOOL_DEPS = @LIBTOOL_DEPS@
libtool: $(LIBTOOL_DEPS)
	$(SHELL) ./config.status --recheck

if ENABLE_DEMOS
SUBDIRS = include drawstuff ode
else
SUBDIRS = include ode
endif

SUBDIRS += OPCODE GIMPACT

bin_SCRIPTS = ode-config

# Utility rule for making a release
release: dist dist-bz2 dist-zip
	@echo Created release packages for ${PACKAGE}-${VERSION}.

EXTRA_DIST = autogen.sh build \
        CHANGELOG.txt INSTALL.txt README.txt LICENSE.TXT





configexecincludedir = $(libdir)/ode/include


install-exec-local: odeconfig.h
	$(mkinstalldirs) $(DESTDIR)$(configexecincludedir)
	file=$(DESTDIR)$(configexecincludedir)/odeconfig.h; \
	if test -r $$file && cmp -s odeconfig.h $$file; then :; \
	else $(INSTALL_DATA) odeconfig.h $$file; fi

uninstall-local:
	rm -f $(DESTDIR)$(configexecincludedir)/odeconfig.h

BUILT_SOURCES = stamp-gc-h #note: not odeconfig.h
odeconfig.h: stamp-gc-h
	@if test -f odeconfig.h; then :; \
	else rm -f stamp-gc-h; $(MAKE) stamp-gc-h; fi
stamp-gc-h: config.status
	cd $(top_builddir) && $(SHELL) ./config.status odeconfig.h
	echo timestamp > stamp-gc-h


DISTCLEANFILES = odeconfig.h stamp-gc-h


