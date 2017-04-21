include Makefile.defs

ARFLAGS=$(MSP432LIB)
LINKFILE=$(MSP432OUT)

${COMPILER}${SUFFIX}/%.out:
	if [ 'x${VERBOSE}' = x ];                                            \
	then                                                                 \
		echo "  LD          ${@} ${LNK_SCP}";                            \
	else                                                                 \
		echo ${LD} -T $${ldname}                                         \
			${LDFLAGS} -o ${@} $(filter %.o %.a, ${^})                   \
			'${LIBM}' '${LIBC}' '${LIBGCC}';                             \
	fi;                                                                  \
	${LD} -T $${ldname}                                                  \
		${LDFLAGS} -o ${@} $(filter %.o %.a, ${^})                       \
		'${LIBM}' '${LIBC}' '${LIBGCC}'
	@${OBJCOPY} -O binary ${@} ${@:.out=.bin}


$(OBJ_401): %.o: %.c
	@echo "  $(COMPILER)-CC      ${<}"
	@${CC} ${CFLAGS_401} $<

$(OBJ_4111): %.o: %.c
	@echo "  $(COMPILER)-CC      ${<}"
	@${CC} ${CFLAGS_4111} ${<}

clean:
	@rm -rf *.o
	@rm -rf *.d
	@rm -rf *.out
	@rm -rf *.a
	@rm -rf *.lib

$(BUILDDIR):
	mkdir $(BUILDDIR)


