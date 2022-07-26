
void _mos_base_init(void);
void _mos_base_fini(void);

extern void _mos_printf_init(void);
extern void _mos_malloc_init(void);
extern void _mos_malloc_fini(void);
void
_mos_base_init() {

	_mos_malloc_init();
	_mos_printf_init();
}

void
_mos_base_fini() {

	_mos_malloc_fini();
}
