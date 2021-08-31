/* TODO


- ensure loop iterations > 1

- set other dac for amplitude modulation. Hmm.

- bit shift the GPIO output bits around?

- routine to calculate total duration/verify completion and calculate the number of 
  data points the function will return. If there's a break of less than 100 samples between
  acquisitions, we might not know for sure how many data points are collected.

- my read data timeout is messed up... and timeout requested should be for a whole program 

- interlock communications - get responses...

- test, test, test




*/
#include "due-nmr-lib.h"
#include <unistd.h>
#include <sys/time.h>
#include <sys/file.h>
#include <math.h>
#include <stdio.h>
#include <termios.h>
#include <string.h>
int due_nmr_init_program(due_nmr_prog_t *program){
  int i;
  program->gindex = 0;
  program->tindex = 0;
  program->recindex = 0;
  program->first_rf_seen = 0;
  program->gsubs = 0;
  program->tsubs = 0;
  program->rsubs = 0;
  program->state = DUE_NMR_STATE_INITIALIZED;
  program->error = 0;
  for (i=0;i<MAX_SUBS;i++){
    program->gsub_addresses[i] = 0;
    program->tsub_addresses[i] = 0;
    program->rsub_addresses[i] = 0;
  }
  
  return 0;
}


int due_nmr_add_event(due_nmr_prog_t *program, unsigned int outputs, unsigned int tx,
		      double phase, unsigned int rx, double duration, char phase_reset){

  unsigned int ticks;
  if (program->state == DUE_NMR_STATE_INITIALIZED) program->state = DUE_NMR_STATE_LOADING;
  if (program->state != DUE_NMR_STATE_LOADING) return -3;
  
  if (program->gindex > MAX_EVENT_WORDS-2 || program->tindex > MAX_EVENT_WORDS-2 || program->recindex > MAX_EVENT_WORDS-2){
    printf("too many words\n");
    return -1;
  }
  ticks = (unsigned int) round(duration * TICKSPERSAMPLE * 500000);
  //  if rx or tx is on, must be a multiple of 2us
  if (rx || tx){
    ticks = (ticks/TICKSPERSAMPLE)*TICKSPERSAMPLE;
  }

  // GPIO:
  program->gprogram[program->gindex++] = ticks;
  program->gprogram[program->gindex++] = outputs;

  // TX:
  program->topcodes[program->tindex] = 0;
  
  // now add the opcode for transmit in:
  if (phase_reset)
    program->topcodes[program->tindex] |= OPCODE_PHASE_RESET;
  if (tx)
    program->topcodes[program->tindex] |= RF_ON;
  program->tprogram[program->tindex++] = ticks;
  if (tx)
    program->tprogram[program->tindex++] = round(phase/180*(1<<31));

  // RX:
  program->ropcodes[program->recindex] = 0;
  // now add the opcode for transmit in:
  if (phase_reset)
    program->ropcodes[program->recindex] |= OPCODE_PHASE_RESET;
  if (rx)
    program->ropcodes[program->recindex] |= RF_ON;
  program->rprogram[program->recindex++] = ticks;
  return 0;
}

int due_nmr_add_new_freq(due_nmr_prog_t *program, unsigned int outputs, unsigned int tx,
			 double phase, unsigned int rx, double duration, double freq){
  uint32_t ftw;


  if (program->state == DUE_NMR_STATE_INITIALIZED) program->state = DUE_NMR_STATE_LOADING;
  if (program->state != DUE_NMR_STATE_LOADING) return -3;
  
  if (program->gindex > MAX_EVENT_WORDS-2 || program->tindex > MAX_EVENT_WORDS-2 || program->recindex > MAX_EVENT_WORDS-2){
    printf("too many words\n");
    return -1;
  }

  ftw = round((freq*(double)(1ULL<<32))/500000.);

  due_nmr_add_event(program, outputs, tx, phase, rx, duration,0);

  // the fiddle with opcodes for rx and tx, and add the ftw.
  program->ropcodes[program->recindex-1] |= OPCODE_SET_FREQ;
  program->rprogram[program->recindex++] = ftw;
  program->topcodes[program->tindex-1-tx] |= OPCODE_SET_FREQ;
  program->tprogram[program->tindex++] = ftw;

  return 0;
}
uint32_t gpio_word(unsigned int outputs,unsigned int opcode){// bit shift the opcode for gPIO word.

  outputs = outputs | ((opcode & 3) << 10) | ((opcode & 4)<<20);
  return outputs;
}

int due_nmr_start_loop(due_nmr_prog_t *program, unsigned int outputs, unsigned int tx,
		       double phase, unsigned int rx, double duration, unsigned int loop_count){

  unsigned int ticks;
  if (program->state == DUE_NMR_STATE_INITIALIZED) program->state = DUE_NMR_STATE_LOADING;
  if (program->state != DUE_NMR_STATE_LOADING) return -3;

  if (program->gindex > MAX_EVENT_WORDS-3 || program->tindex > MAX_EVENT_WORDS-3 || program->recindex > MAX_EVENT_WORDS-3){
    printf("too many words\n");
    return -1;
  }
  ticks = (unsigned int) round(duration * TICKSPERSAMPLE * 500000);
  //  if rx or tx is on, must be a multiple of 2us
  if (rx || tx){
    ticks = (ticks/TICKSPERSAMPLE)*TICKSPERSAMPLE;
  }

  // split opcode up for GPIO
  
  // GPIO:
  program->gprogram[program->gindex++] = ticks;
  program->gprogram[program->gindex++] = gpio_word(outputs, OPCODE_LOOP_START);
  program->gprogram[program->gindex++] = loop_count;
  
  // TX:
  program->topcodes[program->tindex] = OPCODE_LOOP_START;
  // now add the opcode for transmit in:
  if (tx)
    program->topcodes[program->tindex] |= RF_ON;
  program->tprogram[program->tindex++] = ticks;
  if (tx)
    program->tprogram[program->tindex++] = round(phase/180*(1<<31));
  program->tprogram[program->tindex++] = loop_count;
  
  // RX:
  program->ropcodes[program->recindex] = OPCODE_LOOP_START ;
  // now add the opcode for receive in:
  if (rx)
    program->ropcodes[program->recindex] |= RF_ON;
  program->rprogram[program->recindex++] = ticks;
  program->rprogram[program->recindex++] = loop_count;
  return 0;
}


int due_nmr_end_loop(due_nmr_prog_t *program, unsigned int outputs, unsigned int tx,
		     double phase, unsigned int rx,  double duration){
  unsigned int ticks;
  if (program->state == DUE_NMR_STATE_INITIALIZED) program->state = DUE_NMR_STATE_LOADING;
  if (program->state != DUE_NMR_STATE_LOADING) return -3;

  if (program->gindex > MAX_EVENT_WORDS-3 || program->tindex > MAX_EVENT_WORDS-3 || program->recindex > MAX_EVENT_WORDS-3){
    printf("too many words\n");
    return -1;
  }
  ticks = (unsigned int) round(duration * TICKSPERSAMPLE * 500000);
  //  if rx or tx is on, must be a multiple of 2us
  if (rx || tx){
    ticks = (ticks/TICKSPERSAMPLE)*TICKSPERSAMPLE;
  }

  // split opcode up for GPIO
  
  // GPIO:
  program->gprogram[program->gindex++] = ticks;
  program->gprogram[program->gindex++] = gpio_word(outputs, OPCODE_LOOP_END);
  
  // TX:
  program->topcodes[program->tindex] = OPCODE_LOOP_END;
  // now add the opcode for transmit in:
  if (tx)
    program->topcodes[program->tindex] |= RF_ON;
  program->tprogram[program->tindex++] = ticks;
  if (tx)
    program->tprogram[program->tindex++] = round(phase/180*(1<<31));
  
  // RX:
  program->ropcodes[program->recindex] = OPCODE_LOOP_END;
  // now add the opcode for transmit in:
  if (rx)
    program->ropcodes[program->recindex] |= RF_ON;
  program->rprogram[program->recindex++] = ticks;
  return 0;
}


int due_nmr_call_sub(due_nmr_prog_t *program, unsigned int outputs, unsigned int tx,
		     double phase, unsigned int rx, double duration, unsigned int subroutine_id){
  
  unsigned int ticks;
  if (program->state == DUE_NMR_STATE_INITIALIZED) program->state = DUE_NMR_STATE_LOADING;
  if (program->state != DUE_NMR_STATE_LOADING) return -3;

  if (program->gindex > MAX_EVENT_WORDS-3 || program->tindex > MAX_EVENT_WORDS-3 || program->recindex > MAX_EVENT_WORDS-3){
    printf("too many words\n");
    return -1;
  }
  ticks = (unsigned int) round(duration * TICKSPERSAMPLE * 500000);
  //  if rx or tx is on, must be a multiple of 2us
  if (rx || tx){
    ticks = (ticks/TICKSPERSAMPLE)*TICKSPERSAMPLE;
  }

  // split opcode up for GPIO
  
  // GPIO:
  program->gprogram[program->gindex++] = ticks;
  program->gprogram[program->gindex++] = gpio_word(outputs, OPCODE_CALL_SUB);
  program->gprogram[program->gindex++] = subroutine_id; // to be filled in XXX TODO
  
  // TX:
  program->topcodes[program->tindex] = OPCODE_CALL_SUB;
 
  // now add the opcode for transmit in:
  if (tx)
    program->topcodes[program->tindex] |= RF_ON;
  program->tprogram[program->tindex++] = ticks;
  if (tx)
    program->tprogram[program->tindex++] = round(phase/180*(1<<31));
  program->tprogram[program->tindex++] = subroutine_id;
  
  // RX:
  program->ropcodes[program->recindex] = OPCODE_CALL_SUB;
  // now add the opcode for transmit in:
  if (rx)
    program->ropcodes[program->recindex] |= RF_ON;
  program->rprogram[program->recindex++] = ticks;
  program->rprogram[program->recindex++] = subroutine_id;
  return 0;
}

int due_nmr_last_event(due_nmr_prog_t *program, unsigned int outputs, unsigned int tx,
		       double phase, unsigned int rx,  double duration){
  unsigned int ticks;
  if (program->state == DUE_NMR_STATE_INITIALIZED) program->state = DUE_NMR_STATE_LOADING;
  if (program->state != DUE_NMR_STATE_LOADING) return -3;

  if (program->gindex > MAX_EVENT_WORDS-3 || program->tindex > MAX_EVENT_WORDS-3 || program->recindex > MAX_EVENT_WORDS-3){
    printf("too many words\n");
    return -1;
  }
  ticks = (unsigned int) round(duration * TICKSPERSAMPLE * 500000);
  //  if rx or tx is on, must be a multiple of 2us
  if (rx || tx){
    ticks = (ticks/TICKSPERSAMPLE)*TICKSPERSAMPLE;
  }

  // split opcode up for GPIO
  
  // GPIO:
  program->gprogram[program->gindex++] = ticks;
  program->gprogram[program->gindex++] = gpio_word(outputs, OPCODE_LAST_EVENT);
  
  // TX:
  program->topcodes[program->tindex] = OPCODE_LAST_EVENT;
  // now add the opcode for transmit in:
  if (tx)
    program->topcodes[program->tindex] |= RF_ON;
  program->tprogram[program->tindex++] = ticks;
  if (tx)
    program->tprogram[program->tindex++] = round(phase/180*(1<<31));
  
  // RX:
  program->ropcodes[program->recindex] = OPCODE_LAST_EVENT;
  // now add the opcode for transmit in:
  if (rx)
    program->ropcodes[program->recindex] |= RF_ON;
  program->rprogram[program->recindex++] = ticks;
  program->state = DUE_NMR_STATE_SUBS;
  return 0;
}


int due_nmr_start_sub(due_nmr_prog_t *program,  unsigned int subroutine_id){ // no event - next is first of subroutine.
  if (program->state != DUE_NMR_STATE_SUBS) return -3;

  if (program->gsub_addresses[subroutine_id] != 0){
    printf("Trying to redefine a subroutine? Aborted\n");
    return -2;
  }
  program->gsub_addresses[subroutine_id] = program->gindex;
  program->tsub_addresses[subroutine_id] = program->tindex;
  program->rsub_addresses[subroutine_id] = program->recindex;
  return 0;
}

int due_nmr_return(due_nmr_prog_t *program, unsigned int outputs, unsigned int tx,
		   double phase, unsigned int rx,  double duration){
  unsigned int ticks;
  if (program->state == DUE_NMR_STATE_INITIALIZED) program->state = DUE_NMR_STATE_LOADING;
  if (program->state != DUE_NMR_STATE_LOADING) return -3;

  if (program->gindex > MAX_EVENT_WORDS-3 || program->tindex > MAX_EVENT_WORDS-3 || program->recindex > MAX_EVENT_WORDS-3){
    printf("too many words\n");
    return -1;
  }
  ticks = (unsigned int) round(duration * TICKSPERSAMPLE * 500000);
  //  if rx or tx is on, must be a multiple of 2us
  if (rx || tx){
    ticks = (ticks/TICKSPERSAMPLE)*TICKSPERSAMPLE;
  }

  // split opcode up for GPIO
  
  // GPIO:
  program->gprogram[program->gindex++] = ticks;
  program->gprogram[program->gindex++] = gpio_word(outputs, OPCODE_RETURN);
  
  // TX:
  program->topcodes[program->tindex] = OPCODE_RETURN;
  // now add the opcode for transmit in:
  if (tx)
    program->topcodes[program->tindex] |= RF_ON;
  program->tprogram[program->tindex++] = ticks;
  if (tx)
    program->tprogram[program->tindex++] = round(phase/180*(1<<31));
  
  // RX:
  program->ropcodes[program->recindex] = OPCODE_RETURN;
  // now add the opcode for transmit in:
  if (rx)
    program->ropcodes[program->recindex] |= RF_ON;
  program->rprogram[program->recindex++] = ticks;
  return 0;
}


int due_nmr_finalize_program(due_nmr_prog_t *program){
  /* We have a few things to do:
     1) check alignment of rf events, need to be on 2us boundaries
     2) collapse events to shorten the program space
     3) resolve subroutine calls
     4) convert tx and rx event tick counts to 2us ticks, and incorporate opcodes.
     5) verify validity (?)

     1 seems the hardest...
  */
  int i;
  int gindex=0,tindex=0,recindex=0;
  int ngindex,ntindex, nrecindex;
  uint32_t opcode,nopcode,opcode2;
  uint64_t gticks = 0,tticks=0,rticks=0;
  uint32_t to_insert;
  if (program->state != DUE_NMR_STATE_SUBS){
    printf("program has no last event\n");
    return -3;
  }
  // at this point, events in all three different programs should be lined up,
  // though event numbers won't be, because of phase in tx events.

  /* The plan is to
  a) find the first event with tx or rx
  b) lengthen the first event so that first tx/rx event is on a 2us boundary
  c) then go through the rest and lengthen events immediately where boundary
  alignment is needed.
  */

  
  // a) align
  gticks = 0;
  tticks = 0;
  rticks = 0;
  gindex = 0;
  tindex = 0;
  recindex = 0;
  while(gindex < program->gindex){
    // go through events to try to align.
    printf("first align, event with ticks: %i %i %i\n", program->gprogram[gindex],program->tprogram[tindex],program->rprogram[recindex]);
    gticks += program->gprogram[gindex];
    tticks += program->tprogram[tindex];
    rticks += program->rprogram[recindex];
    // now advance to next event
    // those ticks take us to the start of the next event.
    // to advance: gprogram: loop start, and sub call go 3 steps, all others 2
    // tprogram: advance 1, if tx is on, add one. if loop start or sub call add one.
    // rprogram: loop start and sub call are 2 steps, others 1.

    opcode = program->topcodes[tindex] & 7;
    ngindex = gindex + 2;
    ntindex = tindex + 1;
    nrecindex = recindex + 1;
    if ((opcode == OPCODE_LOOP_START )|| (opcode == OPCODE_CALL_SUB) || (opcode == OPCODE_SET_FREQ)){
	ngindex += 1;
	ntindex += 1;
	nrecindex += 1;
    }
    if (program->topcodes[tindex] & RF_ON) 
      ntindex += 1;
    
    // move on to next event.
    tindex = ntindex;
    recindex = nrecindex;
    gindex = ngindex;
      
    if ((program->topcodes[tindex] & RF_ON) || (program->ropcodes[recindex] & RF_ON)){ // found it.
      // how many ticks so far?
      printf("Found a tx or rx event with %li %li %li ticks\n", gticks,tticks,rticks);
      // add in ticks at start:
      to_insert = tticks - (tticks /TICKSPERSAMPLE_ULL)*TICKSPERSAMPLE_ULL; // this is how many ticks are left over after most recent boundary. want to round up.
      to_insert = (TICKSPERSAMPLE - to_insert)%TICKSPERSAMPLE;
      printf("Will insert %i ticks into first event\n",to_insert);
      program->tprogram[0] += to_insert;
      program->rprogram[0] += to_insert;
      program->gprogram[0] += to_insert;
      // make sure we didn't overflow
      if (program->tprogram[0] < to_insert){
	printf("overflowed first event!\n");
	program->error = 1;
	return -4;
      }
      if (program->rprogram[0] < to_insert){
	printf("overflowed first event!\n");
	program->error = 1;
	return -4;
      }
      if (program->gprogram[0] < to_insert){
	printf("overflowed first event!\n");
	program->error = 1;
	return -4;
      }
      gindex = program->gindex; // get out of loop
    }
  } // ok, so first tx/rx event should now be aligned. Start over

  gticks = 0;
  tticks = 0;
  rticks = 0;
  gindex = 0;
  tindex = 0;
  recindex = 0;
  while(gindex < program->gindex){
    // go through events to try to align.
    gticks += program->gprogram[gindex];
    tticks += program->tprogram[tindex];
    rticks += program->rprogram[recindex];
    printf("gindex: %i %i %i, ticks: %i %i %i\n",gindex, tindex,recindex,program->gprogram[gindex], program->tprogram[tindex],program->rprogram[recindex]);
    // now advance to next event
    // those ticks take us to the start of the next event.
    // to advance: gprogram: loop start, and sub call go 3 steps, all others 2
    // tprogram: advance 1, if tx is on, add one. if loop start or sub call add one.
    // rprogram: loop start and sub call are 2 steps, others 1.

    opcode = program->topcodes[tindex] & 7;
    ngindex = gindex + 2;
    ntindex = tindex + 1;
    nrecindex = recindex + 1;
    if ((opcode == OPCODE_LOOP_START )|| (opcode == OPCODE_CALL_SUB) ||(opcode == OPCODE_SET_FREQ)){
      ngindex += 1;
      ntindex += 1;
      nrecindex += 1;
    }
    if (program->topcodes[tindex] & RF_ON) 
      ntindex += 1;

    // are we starting a phase_reset, loop start,  rf transition or freq set event ?
    if (((program->topcodes[ntindex] & OPCODE_PHASE_RESET) == OPCODE_PHASE_RESET) || ((program->ropcodes[nrecindex] & OPCODE_PHASE_RESET) == OPCODE_PHASE_RESET) ||
	((program->topcodes[ntindex] & OPCODE_LOOP_START) == OPCODE_LOOP_START) || ((program->topcodes[ntindex] & OPCODE_SET_FREQ) == OPCODE_SET_FREQ) ||
	((program->topcodes[ntindex] & RF_ON) != (program->topcodes[tindex] & RF_ON)) ||
	((program->ropcodes[nrecindex] & RF_ON) != (program->ropcodes[recindex] & RF_ON))||
	((program->topcodes[tindex] & OPCODE_CALL_SUB) == OPCODE_CALL_SUB) || ((program->topcodes[tindex] & OPCODE_LOOP_END) == OPCODE_LOOP_END) ||
	 ((program->topcodes[tindex] & OPCODE_RETURN) == OPCODE_RETURN)){

      // previous event needs to be lengthened.
      
      to_insert = tticks - (tticks /TICKSPERSAMPLE_ULL)*TICKSPERSAMPLE_ULL; // this is how many ticks are left over after most recent boundary. want to round up.
      if (to_insert > 0){
	printf("alignment, found %li excess ticks\n",to_insert);
	to_insert = (TICKSPERSAMPLE - to_insert)%TICKSPERSAMPLE_ULL;
	printf("Will insert %i ticks into event\n",to_insert);
	program->tprogram[tindex] += to_insert;
	program->rprogram[recindex] += to_insert;
	program->gprogram[gindex] += to_insert;
	tticks += to_insert;
	rticks += to_insert;
	gticks += to_insert;
      // make sure we didn't overflow
	if (program->tprogram[tindex] < to_insert){
	printf("overflowed first event!\n");
	program->error = 1;
	return -4;
	}
	if (program->rprogram[recindex] < to_insert){
	  printf("overflowed first event!\n");
	  program->error = 1;
	  return -4;
	}
	if (program->gprogram[gindex] < to_insert){
	  printf("overflowed first event!\n");
	  program->error = 1;
	  return -4;
	}
      }
    }
      // move on to next event.
    tindex = ntindex;
    recindex = nrecindex;
    gindex = ngindex;
    /*
    //    if (gindex+2 == program->gindex||gindex+3 == program->gindex){ // final event in table
    if (recindex +1 == program->recindex){ //If last event has a freq_set, this won't find it XXX FIXME.
      printf("alignment, looking at last event\n");
      // do this event.
      to_insert = tticks - (tticks /TICKSPERSAMPLE_ULL)*TICKSPERSAMPLE_ULL; // this is how many ticks are left over after most recent boundary. want to round up.
      if (to_insert > 0){
	to_insert = (TICKSPERSAMPLE_ULL - to_insert)%TICKSPERSAMPLE_ULL;
	printf("Will insert %i ticks into final event\n",to_insert);
	program->tprogram[tindex] += to_insert;
	program->rprogram[recindex] += to_insert;
	program->gprogram[gindex] += to_insert;
      // make sure we didn't overflow
	if (program->tprogram[tindex] < to_insert){
	printf("overflowed first event!\n");
	program->error = 1;
	return -4;
	}
	if (program->rprogram[recindex] < to_insert){
	  printf("overflowed first event!\n");
	  program->error = 1;
	  return -4;
	}
	if (program->gprogram[gindex] < to_insert){
	  printf("overflowed first event!\n");
	  program->error = 1;
	  return -4;
	}
	} 
	} */
  } // alignment should be done!
  
  /* now run through all three programs separately and collapse where possible.  
     The rules are:
     1a) Loop start,  phase reset can't be collapsed backwards at all
     1b) last event, loop end, return, call sub can be collapsed backwards into a previous continue
     1c) continue can be collapsed back into a continue, loop start, phase reset.
     
     2a) gpio events can be collapsed if the output word is the same
     2b) tx events can be collapsed if the tx is on or off in both, and if on, has same phase
     2c) rx events can be collapsed if the rx is on or off in both.
  */
  // gpio events first.
  
  gindex = 0;
  opcode = ((program->gprogram[gindex+1]>>10) &3) | ((program->gprogram[gindex+1]>>18) & 4);
  while (gindex < program->gindex){
    int deltaw,i;
    // look at the next event
    ngindex = gindex + 2;
    if (opcode == OPCODE_CALL_SUB || opcode == OPCODE_LOOP_START)
      ngindex += 1;
    nopcode = ((program->gprogram[ngindex+1]>>10) &3) | ((program->gprogram[ngindex+1]>>18) & 4);
    if ( // case 1b:
	(((nopcode == OPCODE_LAST_EVENT) || (nopcode == OPCODE_LOOP_END) ||(nopcode == OPCODE_RETURN) || (nopcode == OPCODE_CALL_SUB)) &&
	 (opcode == OPCODE_CONTINUE)) ||
	//case 1c:
	((nopcode == OPCODE_CONTINUE) && ((opcode == OPCODE_CONTINUE)||(opcode == OPCODE_LOOP_START)|| (opcode == OPCODE_PHASE_RESET)) ) ){
      if( (program->gprogram[gindex+1] & OUTPUT_WORD_MASK) == (program->gprogram[ngindex+1] & OUTPUT_WORD_MASK)){
	if (program->gprogram[gindex] + program->gprogram[ngindex] > program->gprogram[gindex]){ // won't overflow timer count
	  // do the collapse. Timer first
	  //	  printf("Collapsing opcode %i back into %i for gpio\n",nopcode,opcode);
	  program->gprogram[gindex] += program->gprogram[ngindex];
	  // the slide forward remaining events
	  // how many words is the shift?
	  deltaw = 2;
	  if (nopcode == OPCODE_CALL_SUB) deltaw += 1;
	  if (nopcode > OPCODE_CONTINUE){// fix up opcode in the merged event.
	    program->gprogram[gindex+1] = program->gprogram[ngindex+1];
	  }
	  else{ // or keep the old opcode
	    nopcode = opcode;
	  }
	  for (i=ngindex;i< program->gindex-deltaw;i++)
	    program->gprogram[i] = program->gprogram[i+deltaw];
	  ngindex = gindex;	  
	  program->gindex -= deltaw;
	  
	}
      }
    }
    gindex = ngindex;
    opcode = nopcode;
  }

  // now collapse transmit events:
  tindex = 0;
  opcode = program->topcodes[tindex];
  while (tindex < program->tindex){
    int deltaw,i;
    ntindex = tindex + 1;
    if (opcode & RF_ON)
      ntindex += 1;
    if ((opcode&7) == OPCODE_CALL_SUB || (opcode&7)== OPCODE_LOOP_START || (opcode&7) == OPCODE_SET_FREQ)
      ntindex += 1;
    nopcode = program->topcodes[ntindex];
    if ( // case 1b:
	((((nopcode&7) == OPCODE_LAST_EVENT) || ((nopcode&7) == OPCODE_LOOP_END) ||((nopcode&7) == OPCODE_RETURN) ||
	  ((nopcode&7) == OPCODE_CALL_SUB)) &&
	 ((opcode&7) == OPCODE_CONTINUE)) ||
	//case 1c:
	(((nopcode&7) == OPCODE_CONTINUE) && (((opcode&7) == OPCODE_CONTINUE)||((opcode&7) == OPCODE_LOOP_START)||
					      ((opcode&7) == OPCODE_PHASE_RESET) || ((opcode&7) == OPCODE_SET_FREQ)) ) ){
      if ((program->topcodes[tindex] & RF_ON) == (program->topcodes[ntindex] & RF_ON)){ // rf state is same
	if ((program->topcodes[tindex] & RF_ON)  == 0 || program->tprogram[tindex+1] == program->tprogram[ntindex+1]){ //rf is off, or phase is same
	  if (program->tprogram[tindex] + program->tprogram[ntindex] > program->tprogram[tindex]){ // won't overflow timer count
	    printf("Collapsing opcode %i back into %i for tx at index %i %i\n",nopcode,opcode,tindex,ntindex);
	    program->tprogram[tindex] += program->tprogram[ntindex];

	    // how many words is the shift?
	    deltaw = 1;
	    if ((nopcode&7) == OPCODE_CALL_SUB) deltaw += 1;
	    if (opcode & RF_ON) deltaw += 1;

	    // fix up opcode in the merged event.
	    if ((nopcode&7) > OPCODE_CONTINUE){
	      program->topcodes[tindex] = program->topcodes[ntindex];
	    }
	    else{ // or keep the old opcode
	      nopcode = opcode;
	    }

	    // then slide back remaining events
	    
	    for (i=ntindex;i< program->tindex-deltaw;i++){
	      program->tprogram[i] = program->tprogram[i+deltaw];
	      program->topcodes[i] = program->topcodes[i+deltaw];
	    }
	    ntindex = tindex;	  
	    program->tindex -= deltaw;
	  }
	}
      }
    }
    tindex = ntindex;
    opcode = nopcode;
  }

  //   /*
    // now collapse receive events:
  recindex = 0;
  opcode = program->ropcodes[recindex];
  while (recindex < program->recindex){
    int deltaw,i;
    nrecindex = recindex + 1;
    if ((opcode&7) == OPCODE_CALL_SUB || (opcode&7)== OPCODE_LOOP_START || (opcode &7) == OPCODE_SET_FREQ)
      nrecindex += 1;
    nopcode = program->ropcodes[nrecindex];
    printf("at index: %i, %i, opcodes: %i, %i\n",recindex,nrecindex,opcode,nopcode);
    if ( // case 1b:
	((((nopcode&7) == OPCODE_LAST_EVENT) || ((nopcode&7) == OPCODE_LOOP_END) ||((nopcode&7) == OPCODE_RETURN) ||
	  ((nopcode&7) == OPCODE_CALL_SUB)) &&
	 ((opcode&7) == OPCODE_CONTINUE)) ||
	//case 1c:
	(((nopcode&7) == OPCODE_CONTINUE) && (((opcode&7) == OPCODE_CONTINUE)||((opcode&7) == OPCODE_LOOP_START)||
					      ((opcode&7) == OPCODE_PHASE_RESET)||((opcode&7) == OPCODE_SET_FREQ)) ) ){
      printf("passed first check for collapse\n");
      if ((program->ropcodes[recindex] & RF_ON) == (program->ropcodes[nrecindex] & RF_ON)){ // rf state is same
	printf("passed second check for collapse\n");
	if (program->rprogram[recindex] + program->rprogram[nrecindex] > program->rprogram[recindex]){ // won't overflow timer count
	  printf("Collapsing opcode %i back into %i for rx at index: %i %i\n",nopcode,opcode,recindex,nrecindex);
	  program->rprogram[recindex] += program->rprogram[nrecindex];
	  
	  // how many words is the shift?
	  deltaw = 1;
	  if ((nopcode&7) == OPCODE_CALL_SUB) deltaw += 1;
	  
	  // fix up opcode in the merged event.
	  if ((nopcode&7) > OPCODE_CONTINUE){
	    program->ropcodes[recindex] = program->ropcodes[nrecindex];
	  }
	  else{ // or keep the old opcode
	    nopcode = opcode;
	  }
	  
	  // then slide back remaining events
	  printf("deltaw is: %i\n",deltaw);
	  for (i=nrecindex;i< program->recindex-deltaw;i++){
	    program->rprogram[i] = program->rprogram[i+deltaw];
	    program->ropcodes[i] = program->ropcodes[i+deltaw];
	  }
	  nrecindex = recindex;	  
	  program->recindex -= deltaw;
	}
      }
    }
    recindex = nrecindex;
    opcode = nopcode;
    } 
  //  */   
  //     3) resolve subroutine calls
  // gpio first
  gindex = 0;
  while (gindex < program->gindex){
    // look at the next event
    opcode = ((program->gprogram[gindex+1]>>10) &3) | ((program->gprogram[gindex+1]>>18) & 4);
    if (opcode == OPCODE_CALL_SUB){ // resolve it
      printf("found a subroutine call to resolve gpio\n");
	     
      program->gprogram[gindex + 2] = program->gsub_addresses[program->gprogram[gindex+2]];
      if (program->gprogram[gindex+2] == 0){
	printf("subroutine resolution problem? Found address: %i\n",program->gprogram[gindex+2]);
	program->error = 1;
	return -5;
      }
    }

    ngindex = gindex + 2;
    if (opcode == OPCODE_CALL_SUB || opcode == OPCODE_LOOP_START)
      ngindex += 1;

    gindex = ngindex;
  }

  // then transmit
  tindex = 0;
  while (tindex < program->tindex){
    int call_offset;
    // look at the next event
    opcode = program->topcodes[tindex];
    if ((opcode&7) == OPCODE_CALL_SUB){ // resolve it
      printf("found a subroutine call to resolve TX\n");
      call_offset = 1 + ((opcode & RF_ON) == RF_ON);
      program->tprogram[tindex + call_offset] = program->tsub_addresses[program->tprogram[tindex+call_offset]]+program->gindex;
      if (program->rprogram[recindex+call_offset] == 0){
	printf("subroutine resolution problem? Found address: %i\n",program->tprogram[tindex+call_offset]);
	program->error = 1;
	return -5;
      }
    }

    ntindex = tindex + 2;
    if ((opcode&7) == OPCODE_CALL_SUB || (opcode&7) == OPCODE_LOOP_START || (opcode&7) == OPCODE_SET_FREQ)
      ntindex += 1;
    if (opcode & RF_ON)
      ntindex += 1;
    tindex = ntindex;
  }

    // and receive
  recindex = 0;
  while (recindex < program->recindex){
    // look at the next event
    opcode = program->ropcodes[recindex];
    if ((opcode&7) == OPCODE_CALL_SUB){ // resolve it
      printf("found a subroutine call to resolve RX\n");
      program->rprogram[recindex + 1] = program->rsub_addresses[program->rprogram[recindex+1]]+program->gindex+program->tindex;
      if (program->rprogram[recindex+1] == 0){
	printf("subroutine resolution problem? Found address: %i\n",program->rprogram[recindex+1]);
	program->error = 1;
	return -5;
      }
    }

    nrecindex = recindex + 1;
    //   if ((opcode&7) == OPCODE_SET_FREQ)
    //	  printf("at resolve sub calls, at index %i got SET_FREQ, with ftw: %i\n",nrecindex,program->rprogram[nrecindex]); 
    if ((opcode&7) == OPCODE_CALL_SUB || (opcode&7) == OPCODE_LOOP_START || (opcode&7) == OPCODE_SET_FREQ)
      nrecindex += 1;
    recindex = nrecindex;
  }

  //     4) convert tx and rx event tick counts to 2us ticks, and incorporate opcodes.

  tindex = 0;
  while (tindex < program->tindex){
    int call_offset;
    // look at the next event
    opcode = program->topcodes[tindex];
    if((  (program->tprogram[tindex] / TICKSPERSAMPLE) * TICKSPERSAMPLE) != program->tprogram[tindex]){
      printf("got transmit event with %i ticks, non-integral at index %i\n",program->tprogram[tindex],tindex);
      }
    program->tprogram[tindex] = program->tprogram[tindex]/TICKSPERSAMPLE |
      (program->topcodes[tindex] & RF_ON) |
      ((program->topcodes[tindex]&7) << OPCODE_SHIFT);
    //    printf("transmit event dur: %i\n",program->tprogram[tindex] &0x0fffffff);
    
    ntindex = tindex + 1;
    if ((opcode&7) == OPCODE_CALL_SUB || (opcode&7) == OPCODE_LOOP_START || (opcode&7) == OPCODE_SET_FREQ)
      ntindex += 1;
    if (opcode & RF_ON)
      ntindex += 1;
    tindex = ntindex;
  }

    // and receive
  recindex = 0;
  while (recindex < program->recindex){
    // look at the next event
    opcode = program->ropcodes[recindex];
    //    printf("receive event is %i ticks ",program->rprogram[recindex]);
    if((  (program->rprogram[recindex] / TICKSPERSAMPLE) * TICKSPERSAMPLE) != program->rprogram[recindex]){
      printf("got receive event with %i ticks, non-integral at index %i\n",program->rprogram[recindex],recindex);
      }
    program->rprogram[recindex] = program->rprogram[recindex]/TICKSPERSAMPLE |
      (program->ropcodes[recindex] & RF_ON) |
      ((program->ropcodes[recindex]&7) << OPCODE_SHIFT);
    //    printf("receive event dur: %i\n",program->rprogram[recindex] & 0x0fffffff);
    nrecindex = recindex + 1;
    //   if((opcode &7) == OPCODE_SET_FREQ)
    //      printf("rx tick conversion, got set freq at index %i with ftw %i\n",nrecindex,program->rprogram[nrecindex]);
    if ((opcode&7) == OPCODE_CALL_SUB || (opcode&7) == OPCODE_LOOP_START || (opcode&7) == OPCODE_SET_FREQ)
      nrecindex += 1;
    recindex = nrecindex;
  }

  // last, copy transmit and receive in after gpio, checking for length overrun:
  for(i=0;i< program->tindex;i++){
    if (program->gindex+i > MAX_EVENT_WORDS){
      printf("TOO MANY WORDS IN PROGRAM\n");
      program->error = 1;
      return -1;
    }
    program->gprogram[program->gindex+i] = program->tprogram[i];
    
  }
  for(i=0;i< program->recindex;i++){
    if (program->gindex + program->tindex + i > MAX_EVENT_WORDS){
      printf("TOO MANY WORDS IN PROGRAM\n");
      program->error = 1;
      return -1;
    }
    program->gprogram[program->gindex+program->tindex+ i] = program->rprogram[i];
  
  }
  program->state = DUE_NMR_STATE_FINALIZED;
}

int due_nmr_dump_program(due_nmr_prog_t *program){
  uint32_t gindex, tindex, recindex,total;
  int opcode;

  gindex = 0;
  total = 0;
  printf("\nGPIO\nINDEX      Ticks    Outputs   [Type]     [Args]\n");
  while (gindex < program->gindex){
    // look at the next event
    opcode = ((program->gprogram[gindex+1]>>10) &3) | ((program->gprogram[gindex+1]>>18) & 4);
    total += program->gprogram[gindex];
    switch(opcode){
    case OPCODE_CONTINUE:
      printf("%i %i 0x%08x\n", gindex, program->gprogram[gindex], program->gprogram[gindex+1]&OUTPUT_WORD_MASK);
      gindex += 2;
      break;
    case OPCODE_LOOP_START:
      printf("%i %i 0x%08x LOOP_START %i\n", gindex, program->gprogram[gindex], program->gprogram[gindex+1]&OUTPUT_WORD_MASK, program->gprogram[gindex+2]);
      gindex += 3;
      break;
    case OPCODE_LOOP_END:
      printf("%i %i 0x%08x LOOP_END\n", gindex, program->gprogram[gindex], program->gprogram[gindex+1]&OUTPUT_WORD_MASK);
      gindex += 2;
      break;
    case OPCODE_LAST_EVENT:
      printf("%i %i 0x%08x LAST_EVENT\n", gindex, program->gprogram[gindex], program->gprogram[gindex+1]&OUTPUT_WORD_MASK);
      gindex += 2;
      break;
    case OPCODE_CALL_SUB:
      printf("%i %i 0x%08x CALL_SUB %i\n", gindex, program->gprogram[gindex], program->gprogram[gindex+1]&OUTPUT_WORD_MASK, program->gprogram[gindex+2]);
      gindex += 3;
      break;
    case OPCODE_RETURN:
      printf("%i %i 0x%08x LOOP_END\n", gindex, program->gprogram[gindex], program->gprogram[gindex+1]&OUTPUT_WORD_MASK);
      gindex += 2;
      break;
    default:
      printf("Unexpected opcode %i for gpio\n",opcode);

    }
  }
  printf("total ticks: %li, in 2us samples: %i, in seconds: %f\n",total,total/TICKSPERSAMPLE, total/TICKSPERSAMPLE/500000.);

  tindex = 0;
  total = 0;
  printf("\nTX\nINDEX      samples    TX  [Phase]  [Type] [Args]\n");
  while (tindex < program->tindex){
    // look at the next event
    opcode = ((program->tprogram[tindex]>> OPCODE_SHIFT)&7);
    total += program->tprogram[tindex]& 0x0fffffff;
    switch(opcode){
    case OPCODE_CONTINUE:
      if (program->tprogram[tindex] & RF_ON){
	printf("%i %i 1 %f\n", tindex, program->tprogram[tindex]&0x0fffffff, program->tprogram[tindex+1]*180./(1<<31));
	tindex += 2;
      }
      else{
	printf("%i %i 0\n", tindex, program->tprogram[tindex]&0x0fffffff);
	tindex +=1;
      }
      break;
    case OPCODE_LOOP_START:
      if (program->tprogram[tindex] & RF_ON){
	printf("%i %i 1 %f LOOP_START %i\n", tindex, program->tprogram[tindex]&0x0fffffff, program->tprogram[tindex+1]*180./(1<<31),
	       program->tprogram[tindex+2]);
	tindex += 3;
      }
      else{
	printf("%i %i 0 LOOP_START %i\n", tindex, program->tprogram[tindex]&0x0fffffff, program->tprogram[tindex+1]);
	tindex +=2;
      }
      break;
    case OPCODE_LOOP_END:
      if (program->tprogram[tindex] & RF_ON){
	printf("%i %i 1 %f LOOP_END\n", tindex, program->tprogram[tindex]&0x0fffffff, program->tprogram[tindex+1]*180./(1<<31));
	tindex += 2;
      }
      else{
	printf("%i %i 0 LOOP_END\n", tindex, program->tprogram[tindex]&0x0fffffff);
	tindex +=1;
      }
      break;
    case OPCODE_LAST_EVENT:
      if (program->tprogram[tindex] & RF_ON){
	printf("%i %i 1 %f LAST_EVENT\n", tindex, program->tprogram[tindex]&0x0fffffff, program->tprogram[tindex+1]*180./(1<<31));
	tindex += 2;
      }
      else{
	printf("%i %i 0 LAST_EVENT\n", tindex, program->tprogram[tindex]&0x0fffffff);
	tindex +=1;
      }
      break;
    case OPCODE_CALL_SUB:
      if (program->tprogram[tindex] & RF_ON){
	printf("%i %i 1 %f CALL_SUB %i\n", tindex, program->tprogram[tindex]&0x0fffffff, program->tprogram[tindex+1]*180./(1<<31),
	       program->tprogram[tindex+2]);
	tindex += 3;
      }
      else{
	printf("%i %i 0 CALL_SUB %i\n", tindex, program->tprogram[tindex]&0x0fffffff, program->tprogram[tindex+1]);
	tindex +=2;
      }
      break;
    case OPCODE_RETURN:
      if (program->tprogram[tindex] & RF_ON){
	printf("%i %i 1 %f RETURN\n", tindex, program->tprogram[tindex]&0x0fffffff, program->tprogram[tindex+1]*180./(1<<31));
	tindex += 2;
      }
      else{
	printf("%i %i 0 RETURN\n", tindex, program->tprogram[tindex]&0x0fffffff);
	tindex +=1;
      }
      break;
    case OPCODE_PHASE_RESET:
      if (program->tprogram[tindex] & RF_ON){
	printf("%i %i 1 %f PHASE_RESET\n", tindex, program->tprogram[tindex]&0x0fffffff, program->tprogram[tindex+1]*180./(1<<31));
	tindex += 2;
      }
      else{
	printf("%i %i 0 PHASE_RESET\n", tindex, program->tprogram[tindex]&0x0fffffff);
	tindex +=1;
      }
      break;
    case OPCODE_SET_FREQ:
      if (program->tprogram[tindex] & RF_ON){
	printf("%i %i 1 %f SET_FREQ %i\n", tindex, program->tprogram[tindex]&0x0fffffff, program->tprogram[tindex+1]*180./(1<<31),
	       program->tprogram[tindex+2]);
	tindex += 3;
      }
      else{
	printf("%i %i 0 SET_FREQ %i\n", tindex, program->tprogram[tindex]&0x0fffffff, program->tprogram[tindex+1]);
	tindex +=2;
      }
      break;
    default:
      printf("Unexpected opcode %i for gpio\n",opcode);

    }
  }
  printf("total 2us samples: %i, in seconds: %f\n",total,total/500000.);


  recindex = 0;
  total = 0;
  printf("\nRX\nINDEX      samples    RX   [Type] [Args]\n");
  while (recindex < program->recindex){
    // look at the next event
    opcode = ((program->rprogram[recindex]>> OPCODE_SHIFT)&7);
    total += program->rprogram[recindex]& 0x0fffffff;
    switch(opcode){
    case OPCODE_CONTINUE:
      printf("%i %i %i\n", recindex, program->rprogram[recindex]&0x0fffffff, (program->rprogram[recindex] & RF_ON)>> 31);
      recindex += 1;
      break;
    case OPCODE_LOOP_START:
      printf("%i %i %i LOOP_START %i\n", recindex, program->rprogram[recindex]&0x0fffffff, (program->rprogram[recindex] & RF_ON)>> 31,
	     program->rprogram[recindex+1]);
      recindex += 2;
      break;
    case OPCODE_LOOP_END:
      printf("%i %i %i LOOP_END\n", recindex, program->rprogram[recindex]&0x0fffffff, (program->rprogram[recindex] & RF_ON)>> 31);
      recindex += 1;
      break;
    case OPCODE_LAST_EVENT:
      printf("%i %i %i LAST_EVENT\n", recindex, program->rprogram[recindex]&0x0fffffff, (program->rprogram[recindex] & RF_ON)>> 31);
      recindex += 1;
      break;
    case OPCODE_CALL_SUB:
      printf("%i %i %i CALL_SUB %i\n", recindex, program->rprogram[recindex]&0x0fffffff, (program->rprogram[recindex] & RF_ON)>> 31,
	     program->rprogram[recindex+1]);
      recindex += 2;
      break;
    case OPCODE_RETURN:
      printf("%i %i %i RETURN\n", recindex, program->rprogram[recindex]&0x0fffffff, (program->rprogram[recindex] & RF_ON)>> 31);
      recindex += 1;
      break;
    case OPCODE_PHASE_RESET:
      printf("%i %i %i PHASE_RESET\n", recindex, program->rprogram[recindex]&0x0fffffff, (program->rprogram[recindex] & RF_ON)>> 31);
      recindex += 1;
      break;
    case OPCODE_SET_FREQ:
      printf("%i %i %i SET_FREQ %i\n", recindex, program->rprogram[recindex]&0x0fffffff, (program->rprogram[recindex] & RF_ON)>>31, program->rprogram[recindex+1]);
      recindex += 2;
      break;
    default:
      printf("Unexpected opcode %i for gpio\n",opcode);

    }
  }
  printf("total 2us samples: %i, in seconds: %f\n",total, total/500000.);
}
//////////////////////////////

int due_nmr_read_data(int fd, int npts,int timeout, int16_t *data, char *flags){
  // read the specified number of data points
  // will return early if there's a timeout or an unexpected message.
  // 
  // npts is the number of complex points. Returned as 16bit signed integers,
  // in data.
  // the data buffer should be able to hold 2*npts 16 bit numbers.

  //  the return value is simple the number of 16bit ints returned.
  // flags bitmask of:
  //        0x01 - got a LAST EVENT
  //        0x02 - read got an error - probably a signal
  //        0x04 - timeout
  //        0x08 - short read. Out of sync.
  //        0x10 - message other than LAST EVENT from due
  //        0x20 - got a SHUTDOWN
  //        0 - success
  //
  int i,pos=0,newbytes,j,bnp;
  uint8_t buff[64];
  *flags = 0;
  for (i=0;i<timeout;i++){
    newbytes = read(fd, buff, 64);
    if (newbytes < 0 ) {// an error - probably a signal
      *flags |= 2;
      return pos;
    }
    if (newbytes > 0 && newbytes != 64){
      printf("PANIC, read %i bytes, not 64\n",newbytes);
      *flags = 8;
      return pos;
    }
    if (newbytes == 64){
      timeout += 1;
      if ( buff[0] & 0x80){ // its numerical data
	//	printf("buffer has %i points\n",buff[2]+buff[3]*256);
	for (j=0;j< buff[2]+buff[3]*256;j++)
	  data[pos++] = (buff[2*j+4]+buff[2*j+5]*256)*((j%2)*2-1);
	
	if( pos == npts*2) return npts; // return the number of points returned - will be 2*npts.
      }
      else{
	// otherwise we got a message:
	printf("Got message: %s\n", buff);
	if (strncmp("LAST EVENT", buff, 10) == 0)
	  *flags |= 1;
	else if (strncmp("SHUTDOWN", buff, 8) == 0)
	  *flags |= 0x20; // we ran out of time
	// any other message is an error and we get out.
	else{
	  *flags |= 0x10;
	  printf("returning from read data with %i points or %i read\n", pos, npts);
	  return pos;
	}
      }
    }

  }
  *flags |= 0x04; //timeout
  return pos;
}

char due_nmr_read_response(int fd, int timeout){
  // intending to get a text response like SHUTDOWN, LAST EVENT, or id string
  // timeout in 0.1s increments

  // responses: 
  char buff[64];
  int newbytes,i;
  for( i=0;i<timeout;i++){
    newbytes = read(fd, buff, 64);
    if (newbytes < 0 ){
      printf("read response got <0\n");
      return -1;
    }
    if (newbytes > 0 && newbytes != 64){
      printf("read response got %i characters\n",newbytes);
      printf(" read response got: %s\n",buff);
      return -1;
    }
    if (newbytes == 64){
      if (strstr(buff, " of ") == NULL)
	printf(" read response got: %s\n",buff);
      if (strncmp("LAST EVENT", buff, 10) == 0) return 1;
      if (strncmp("SHUTDOWN", buff, 8) == 0) return 2;
      if (strncmp("Due NMR v1", buff, 10) == 0) return 3;
      if (strncmp("WATCHDOG", buff,8) == 0) return 4;
      return 0; // other response?
    }
  
        
  }
  printf("read response timeout\n");
  return -1;
}


int due_nmr_download_prog(int fd, due_nmr_prog_t *program){
  char cbyte[7];
  uint32_t hprog[MAX_EVENT_WORDS];
  char buff[64];
  unsigned char c1,c2;
  int c1d,c2d, total;
  int i,bytes_read;
  struct timeval start_time,end_time;
  struct timezone tz;
  double d_time;
  if (fd <= 0){
    printf("dueNMR: due_download: got invalid file descriptor\n");
    return -1;
  }
  if (program->error != 0){
    printf("dueNMR: pulse program has an error flag set, will not download!\n");
    return -1;
  }
  if (program->state != DUE_NMR_STATE_FINALIZED){
    printf("dueNMR: WARNING. Program has not been finalized, aborting\n");
    return -1;
  }
  gettimeofday(&start_time,&tz);

  printf("dueNMR: Sending prog size: %i %i %i ",program->gindex, program->tindex, program->recindex);
  fflush(stdout);
  //  write(fd,"D",1);
  
  // send data length, low byte, high byte
  cbyte[0] = 'D';
  cbyte[1] = program->gindex & 0xff;
  cbyte[2] = (program->gindex>>8)&0xff;
  cbyte[3] = program->tindex & 0xff;
  cbyte[4] = (program->tindex>>8)&0xff;
  cbyte[5] = program->recindex & 0xff;
  cbyte[6] = (program->recindex>>8)&0xff;
  write(fd,cbyte,7);
  due_nmr_read_response(fd, 1);

  
  printf("dueNMR: Sending program: \n");
  fflush(stdout);
  total = program->gindex + program->tindex+program->recindex;
  // first do 64 byte blocks:
  for ( i = 0 ; i+16 < total ; i += 16){
    // printf("64 byte block at pos: %i ",i);
    write(fd,&(program->gprogram[i]),64);
    due_nmr_read_response(fd, 2);
  }
  //write the rest:
  printf("writing final %i words\n",(total-i));
  write(fd,&(program->gprogram[i]),(total-i)*4);
  due_nmr_read_response(fd, 2);

  gettimeofday(&end_time,&tz);

  d_time=(end_time.tv_sec-start_time.tv_sec)*1e6
    +(end_time.tv_usec-start_time.tv_usec);
  fprintf(stderr,"downloaded %i bytes in:  %.0f us\n",total*4,d_time);
  return -1;
}

int due_nmr_run_program(int fd, char start_command){
  // start_command is Y.
  int bytes_read;
  if (fd <= 0){
    printf("dueNMR: due_run: got invalid file descriptor\n");
    return -1;
  }
  printf("dueNMR: writing %c: \n",start_command);
  write(fd,&start_command,1);
  /*
  bytes_read = my_read(fd,sbuff,25);
  if (bytes_read > 0 ){
   printf("duepp: Got: %s",sbuff);
   if (strncmp(sbuff,"Starting",8) == 0 || strncmp(sbuff,"Restarting",10) == 0)
     return 0;
  }
  return -1;
  */
  return 0;
}

/* meh. Just use read_response
int due_wait_for_completion(int fd, int timeout){
  char sbuff[BUFFLEN];
  int bytes_read;
  if (fd <= 0){
    printf("duepp: due_wait_for_completion: got invalid file descriptor\n");
    return -1;
  }
  printf("duepp: waiting for completion: \n");
  fflush(stdout);
  bytes_read = my_read(fd,sbuff,timeout);
  if (bytes_read == -1) return -1; // woken by signal.
  if (bytes_read == 0) return 1; // timeout 
  printf("duepp: Got: %s\n",sbuff);
  if (strncmp(sbuff,"Final Event started",19)==0) return 0;
  if (strncmp(sbuff,"Was interrupted",15)==0) return 2;
  return -1;
}
*/


int due_nmr_interrupt_program(int fd){
  int i;
  // hmm, there might be some stray data??
  printf("dueNMR: writing K: ");
  write(fd,"S",1);
  i = due_nmr_read_response(fd,1); // will collect a last packet if there is one.
  return 0;
}

/*
int due_nmr_get_status(int fd){
  char sbuff[BUFFLEN];
  int bytes_read;
  if (fd <= 0){
    printf("dueNMR: due_nmr_get_status: got invalid file descriptor\n");
    return -1;
  }
  printf("duepp: writing x: \n");
  write(fd,"x",1);
  bytes_read = my_read(fd,sbuff,100);
  if (bytes_read == 0) return -1; //
  printf("duepp: Got: %s\n",sbuff);
  return 0;
}

*/
int due_nmr_set_freq(int fd, double freq){
  uint32_t ftw;
  char sbuff[5];
  if (fd <= 0){
    printf("dueNMR: due_nmr_set_freq: got invalid file descriptor\n");
    return -1;
  }

  ftw = round((freq*(double)(1ULL<<32))/500000.);
  printf("Requested freq: %lf, setting ftw to: %i, will get f: %lf\n",
	 freq, ftw, 500000.*ftw/(double)(1ULL<<32));
  sbuff[0] = 'F';
  sbuff[1] = ftw & 0xff;
  sbuff[2] = (ftw>>8) & 0xff;
  sbuff[3] = (ftw>>16) & 0xff;
  sbuff[4] = (ftw>>24) & 0xff;
  
  write(fd,sbuff,5);
   
}


void due_nmr_close_prog(int fd){
  if (fd >=0){
    flock(fd,LOCK_UN);
    close(fd);
  }
}

int due_nmr_open_prog(char *device){
  // returns the file descriptor if successful
  // -1 otherwise.
  struct termios myterm;
  int fd0,bytes_read,rval;
  char sbuff[64];
  
  fd0 = open(device, O_RDWR | O_NOCTTY);
  if (fd0 < 0){
    printf("dueNMR: can't open port to programmer %s\n",device);
    return -1;
  }
  rval = flock(fd0,LOCK_EX|LOCK_NB); // exclusive lock, don't block if we can't.
  if (rval < 0){
    printf("dueNMR: Couldn't obtain lock on due programmer board\n");
    close(fd0);
    return -1;
  }
  
 tcgetattr(fd0,&myterm);
 myterm.c_iflag = 0;
 myterm.c_oflag= CR0;
 myterm.c_cflag = CS8 |CLOCAL|CREAD|B38400; // speed doesn't matter for usb
 myterm.c_lflag=0;
 myterm.c_cc[VMIN]=0; // non-blocking
 myterm.c_cc[VTIME]=1; // returns after 0.1s if no characters available
 
 tcsetattr(fd0,TCSANOW, &myterm);
 tcflush(fd0,TCIFLUSH);
 
 
 printf("dueNMR: writing Q: ");
 write(fd0,"Q",1);
 bytes_read = due_nmr_read_response(fd0,20);
 if (bytes_read == 3 ){
   return fd0;
 }
 rval=flock(fd0,LOCK_UN);
 close(fd0);
 return -1;
}


int main(){
  // test it out a little:

  due_nmr_prog_t prog;
  int dfd;
  due_nmr_init_program(&prog);
  // add some events
  // prog, outputs, tx, phase, rx, duration, phase_reset
  // tx relay control is D33 - value position 2
  // rx relay contrl is D34 - value position 4
  /*
  due_nmr_add_event(&prog, 0, 0, 0., 0,.01,0);
  due_nmr_add_event(&prog, 2, 0, 0., 0, .002, 0);  // flip tx relay on
  due_nmr_add_event(&prog, 2, 1, 0., 0, .001, 0);  // the pulse
  due_nmr_add_event(&prog, 2, 0, 0., 0, .001, 0);  // keep tx relay on


  due_nmr_add_event(&prog, 0, 0, 0., 0, .001, 0);  // both relays off
  due_nmr_add_event(&prog, 4, 0, 0., 0, .001, 0);  // flip rx relay on

 
  due_nmr_add_event(&prog, 4, 0, 0., 1, .02, 0); // collect 20ms data = 400 point

  due_nmr_last_event(&prog, 0, 0, 0., 0, 0.4);
  */

  double pw90 = 1500e-6, pw180=pw90*2;
  due_nmr_add_event(&prog, 0, 0, 0., 0,.01,0);
  
  due_nmr_add_event(&prog, 2, 0, 0., 0, .002, 0);  // flip tx relay on
  due_nmr_add_event(&prog, 2, 1, 0., 0, pw90, 0);  // the pulse
  due_nmr_add_event(&prog, 2, 0, 0., 0, .005, 0);  // keep tx relay on, echo delay
  due_nmr_add_event(&prog, 2, 1, 0., 0, pw180, 0);  //second pulse
  due_nmr_add_event(&prog, 2, 0, 0., 0, .001, 0);  // keep tx relay on - let pulse ring down.
  due_nmr_add_event(&prog, 0, 0, 0., 0, .001, 0);  // both relays off
  due_nmr_add_event(&prog, 4, 0, 0., 0, .001, 0);  // rx relay on
  due_nmr_add_event(&prog, 4, 0, 0., 1, .02, 0); // collect 20ms data = 400 poin
  
  /* sim tx and rx
  due_nmr_add_event(&prog, 0, 1, 0., 1, .01, 0); // collect 20ms data = 400 poin
  due_nmr_add_event(&prog, 0, 1, 90., 1, .01, 0); // collect 20ms data = 400 poin
  */
  
  due_nmr_last_event(&prog, 0, 0, 0., 0, 0.6);


  due_nmr_finalize_program(&prog);
  due_nmr_dump_program(&prog);
  
  // return 0;
  dfd = due_nmr_open_prog("/dev/cu.usbmodem14101");

#define NPTS 800
#define NSCAN 5000
  
  int i,j;
  int16_t data[NPTS*2];
  int32_t accum[NPTS*2];
  char flags,rval;
  FILE *outfile;
  int dummy = 0;
  
  memset(accum,0,NPTS*2*4);
  printf("open program, fd is: %i\n",dfd);
  due_nmr_set_freq(dfd, 63000.);
  
  due_nmr_read_response(dfd,2);

  for(i=0;i<NSCAN+dummy;i++) {

    //////////////////
    due_nmr_init_program(&prog);
    // add some events
    // prog, outputs, tx, phase, rx, duration, phase_reset
    // tx relay control is D33 - value position 2
    // rx relay contrl is D34 - value position 4
    pw90= 150e-6; // at full power 90 might be 90us.
    due_nmr_add_event(&prog, 0, 0, 0., 0,.01,0);
  
    //    due_nmr_add_event(&prog, 4, 0, 0., 0, .050, 0);  // rx relay on - discharge the tune cap - charged by bias current
    due_nmr_add_event(&prog, 2, 0, 0., 0, .001, 1);  // flip tx relay on. Do phase reset.
    due_nmr_add_event(&prog, 2, 1, 180*(i%2), 0, pw90, 0);  // the pulse
    due_nmr_add_event(&prog, 2, 0, 0., 0, .016666, 0);  // echo
    
    due_nmr_add_event(&prog, 2, 1, 90, 0, pw90*1.6, 0);  // the pulse
    due_nmr_add_event(&prog, 2, 0, 0., 0, .001, 0);  // allow some ring down
    due_nmr_add_event(&prog, 4, 0, 0., 0, .001, 0);  // rx relay on.
    due_nmr_add_event(&prog, 4, 0, 0., 1, .04, 0); // collect 40ms data = 800 poin
   
    due_nmr_last_event(&prog, 0, 0, 0., 0, 5);

    due_nmr_finalize_program(&prog);

    /////////////////

    //       due_nmr_set_freq(dfd, 65700.);
    //read_response(dfd,2);
     
    printf("\nCalling download, for i val: %i\n",i);
    due_nmr_download_prog(dfd, &prog);
    if(i== 0)
      due_nmr_run_program(dfd, 'Y');
    // should check status to make sure its running.
    rval = due_nmr_read_data(dfd, NPTS,120, data, &flags);
    printf("return from read, got flags: %i\n",flags&0xff);
    if (i>dummy-1){
      outfile = fopen("out.txt","w");

      for (j=0;j<NPTS;j++){
	accum[2*j] += ((i%2)*2-1)*data[2*j];
	accum[2*j+1] += ((i%2)*2-1)*data[2*j+1];
	fprintf(outfile,"%i %i\n",accum[2*j],accum[2*j+1]);
      }
      fclose(outfile);
      printf("data done\n");
    }
    if (flags != 0 && flags != 1){
      printf("aborting\n");
      break;
    }
    // read data?
    if ((flags & 0x01) == 0){
      rval = due_nmr_read_response(dfd,20);
      if (rval != 1){
	printf("aborting2, for rval of %i in read_response\n",rval);
	break;
      }
    }
  }
  due_nmr_read_response(dfd,80);
  
  due_nmr_close_prog(dfd);
}
