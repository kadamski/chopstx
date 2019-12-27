struct vrsp;

struct vrsp *vrsp_open (void);

void vrsp_prepare_poll (struct vrsp *v, chopstx_poll_cond_t *poll_desc);
int vrsp_screen_acquire (struct vrsp *v);
void vrsp_screen_release (struct vrsp *v);
