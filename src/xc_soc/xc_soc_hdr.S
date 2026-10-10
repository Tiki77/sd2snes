/* xc_soc.bin header (see Makefile): "XSOC", version 1, bootrom length, additions length, padded with 0xFF to
   512 bytes. BOOTROM_LEN and FW_LEN come from --defsym. */
	.section .data
	.ascii "XSOC"
	.word 1
	.word BOOTROM_LEN
	.word FW_LEN
	.fill 512 - 16, 1, 0xff
