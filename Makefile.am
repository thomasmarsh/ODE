LIBTOOL_DEPS = @LIBTOOL_DEPS@
libtool: $(LIBTOOL_DEPS)
	$(SHELL) ./config.status --recheck
SUBDIRS =   include drawstuff ode 

bin_SCRIPTS = ode-config

# Utility rule for making a release
release: dist dist-bz2 dist-zip
	@echo Created release packages for ${PACKAGE}-${VERSION}.

# Rules for creating .tar.bz2 and .zip packages
dist-bz2: ${PACKAGE}-${VERSION}.tar.gz
	gzip -dc ${PACKAGE}-${VERSION}.tar.gz | bzip2 > ${PACKAGE}-${VERSION}.tar.bz2

dist-zip: ${PACKAGE}-${VERSION}.tar.gz
	tar -zxf ${PACKAGE}-${VERSION}.tar.gz && \
	zip -r ${PACKAGE}-${VERSION}.zip ${PACKAGE}-${VERSION} && \
	rm -rf ${PACKAGE}-${VERSION}

if USE_SONAME
install-exec-hook:
	/sbin/ldconfig
	ln -s $(prefix)/lib/@ODE_SONAME@.@ODE_REVISION@.@ODE_AGE@ \
	$(prefix)/lib/libode.so
	ln -s $(prefix)/lib/@ODE_SONAME@.@ODE_REVISION@.@ODE_AGE@ \
	$(prefix)/lib/@ODE_SONAME@.@ODE_REVISION@
endif
