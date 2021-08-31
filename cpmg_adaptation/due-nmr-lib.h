#include <stdint.h>

#define MAX_EVENT_WORDS 14000
#define TICKSPERSAMPLE 84
#define TICKSPERSAMPLE_ULL 84ULL

#define LOOP_LEVELS 4
#define OUTPUT_WORD_MASK (0x37eff3fe & ~(1 << 23))  // use 23 as FLAG, remove for now


#define OPCODE_CONTINUE 0
#define OPCODE_LOOP_START 1
#define OPCODE_LOOP_END 2
#define OPCODE_LAST_EVENT 3
#define OPCODE_CALL_SUB 4
#define OPCODE_RETURN 5
#define OPCODE_PHASE_RESET 6
#define OPCODE_SET_FREQ 7

#define OPCODE_SHIFT 28
#define RF_ON (1<<31)

#define DUE_NMR_STATE_EMPTY 0
#define DUE_NMR_STATE_INITIALIZED 1
#define DUE_NMR_STATE_LOADING 2
#define DUE_NMR_STATE_SUBS 3
#define DUE_NMR_STATE_FINALIZED 4
#define MAX_SUBS 32


typedef struct due_nmr_prog_type{
  unsigned int gindex, tindex, recindex,
    glooplevel,tlooplevel,rlooplevel,
    gloopinex[LOOP_LEVELS],gloopcount[LOOP_LEVELS],
    tloopinex[LOOP_LEVELS],tloopcount[LOOP_LEVELS],
    rloopinex[LOOP_LEVELS],rloopcount[LOOP_LEVELS];
  uint32_t gprogram[MAX_EVENT_WORDS],tprogram[MAX_EVENT_WORDS],rprogram[MAX_EVENT_WORDS],program[MAX_EVENT_WORDS];
  uint32_t topcodes[MAX_EVENT_WORDS],ropcodes[MAX_EVENT_WORDS];
  uint32_t gsub_addresses[MAX_SUBS], tsub_addresses[MAX_SUBS], rsub_addresses[MAX_SUBS];
  char gsubs, tsubs, rsubs;
  char state;
  char first_rf_seen;
  char error;
  
      

} due_nmr_prog_t;


int due_nmr_init_program(due_nmr_prog_t *program);
int due_nmr_add_event(due_nmr_prog_t *program, unsigned int outputs, unsigned int tx,
		      double phase, unsigned int rx, double duration, char phase_reset);

int due_nmr_start_loop(due_nmr_prog_t *program, unsigned int outputs, unsigned int tx,
		       double phase, unsigned int rx, double duration, unsigned int loop_count);
int due_nmr_end_loop(due_nmr_prog_t *program, unsigned int outputs, unsigned int tx,
		     double phase, unsigned int rx,  double duration);

int due_nmr_call_sub(due_nmr_prog_t *program, unsigned int outputs, unsigned int tx,
		     double phase, unsigned int rx, double duration,  unsigned int subroutine_id);
int due_nmr_last_event(due_nmr_prog_t *program, unsigned int outputs, unsigned int tx,
		       double phase, unsigned int rx,  double duration);
int due_nmr_start_sub(due_nmr_prog_t *program, unsigned int subroutine_id);
int due_nmr_return(due_nmr_prog_t *program, unsigned int outputs, unsigned int tx,
		   double phase, unsigned int rx,  double duration);


int due_nmr_finalize_program(due_nmr_prog_t *program);

int due_nmr_dump_program(due_nmr_prog_t *program);
void due_nmr_close_prog(int fd);
int due_nmr_open_prog(char *device);
int due_nmr_download_prog(int fd, due_nmr_prog_t *program);
int due_run_program(int fd, char start_command);
int due_nmr_interrupt_program(int fd);
int due_nmr_set_freq();
char due_nmr_read_response(int fd, int timeout);
int due_nmr_read_data(int fd, int npts,int timeout, int16_t *data, char *flags);
int due_nmr_run_program(int fd, char start_command);
int due_nmr_add_new_freq(due_nmr_prog_t *program, unsigned int outputs, unsigned int tx,
			 double phase, unsigned int rx, double duration, double freq);
