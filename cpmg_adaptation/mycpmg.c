#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "due-nmr-lib.h"

#define NPTS 20000
#define NECHO 20

char basename[11]="cpmg";

int main(int argc, char *argv[]){

  // expect argv to contain: nscan, pw90 and pw180 (in us)


  
  due_nmr_prog_t prog;
  int dfd;
  double pw90, pw180;
  char name[200];
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

  

  int i,j;
  int16_t data[NPTS*2];
  int32_t accum[NPTS*2];
  char flags,rval;
  FILE *outfile;
  int dummy = 0, nscan, npts;
  double freq = 78000;
  
  if (argc != 3){
    printf("usage: cpmg nscan pw90(all integers, pw's in us\n");
    return 0;
  }

  nscan = atoi(argv[1]);
  pw90 = atoi(argv[2])*1e-6;
  pw180 = pw90*2;
  printf("doing: nscan = %i\npw90 = %g\npw180 = %g\n",nscan,pw90,pw180);

  
  due_nmr_init_program(&prog);
  dfd = due_nmr_open_prog("/dev/cu.usbmodem14201");
  
  memset(accum,0,NPTS*2*4);
  printf("open program, fd is: %i\n",dfd);
  due_nmr_set_freq(dfd, freq);
  
  due_nmr_read_response(dfd,2);

  for(i=0;i<nscan+dummy;i++) {
      //////////////////
	      due_nmr_init_program(&prog);
	    // add some events
	    // prog, outputs, tx, phase, rx, duration, phase_reset
	    // tx relay control is D33 - value position 2
	    // rx relay contrl is D34 - value position 4
	    //    pw90= 150e-6; could be this 
		// at full power 90 might be 90us. ????

	    due_nmr_add_event(&prog, 0, 0, 0., 0,.01,0); //initialize

	    due_nmr_add_event(&prog, 2, 0, 0., 0, .001, 1);  // flip tx relay on. Do phase reset.
	    due_nmr_add_event(&prog, 2, 1, 180*(i%2), 0, pw90, 0);  // the pulse
	    due_nmr_add_event(&prog, 2, 0, 0., 0, .016-pw180/2-.002, 0);  // echo
	    //due_nmr_add_event(&prog, 2, 0, 0., 0, .001, 0);
	    //due_nmr_add_event(&prog, 2, 0, 0., 0, .01-pw180/2-.002, 0);

		for(j=0;j<NECHO;j++){
			
			due_nmr_add_event(&prog, 2, 0, 0., 0, .002, 0);  // flip tx relay on.
			due_nmr_add_event(&prog, 2, 1, 90, 0, pw180, 0);  // the pulse
			due_nmr_add_event(&prog, 2, 0, 0., 0, .001, 0);  // allow some ring down
			due_nmr_add_event(&prog, 0, 0, 0., 0, .001, 0);  // neither
			due_nmr_add_event(&prog, 4, 0, 0., 0, .001, 0);  // rx relay on.
			due_nmr_add_event(&prog, 4, 0, 0., 1, .032-pw180-.005, 0); // collect _32ms data
			//due_nmr_add_event(&prog, 4, 0, 0., 1, .017, 0);
			//due_nmr_add_event(&prog, 4, 0, 0., 0, .015, 0);
			//due_nmr_add_event(&prog, 4, 0, 0., 1, .002, 0);
			//due_nmr_add_event(&prog, 4, 0, 0., 1, .02-pw180-.005, 0);
		}
	      
	      
	    npts = (0.032-pw180-.005)/50e-6;
	    //npts= 0.017/50e-6;
	    //npts= (0.02-pw180-.005)/50e-6;
	    npts = npts*NECHO;
	    due_nmr_last_event(&prog, 0, 0, 0., 0, 1);
	      
	    due_nmr_finalize_program(&prog);
	      

	    if (i== 0)
		due_nmr_dump_program(&prog);
	      
	      
	    printf("\nCalling download, for i val: %i\n",i);
	    due_nmr_download_prog(dfd, &prog);
	      
	    if(i== 0)
		due_nmr_run_program(dfd, 'Y');
	      // should check status to make sure its running.
	    rval = due_nmr_read_data(dfd, npts,120, data, &flags);
	    printf("return from read, got flags: %i\n",flags&0xff);
	    

	    if (i>dummy-1){
			
			sprintf(name,"%s-%i",basename, i);
			outfile = fopen(name,"w");
		

			fprintf(outfile,"# cpmg pw90: %f pw180: %f, scans complete: %i\n",pw90,pw180,i+1);
		
			for (j=0;j<npts;j++){
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
	      

		if ((flags & 0x01) == 0){
			
			rval = due_nmr_read_response(dfd,20);
			
			if (rval != 1){
			  printf("aborting2, for rval of %i in read_response\n",rval);
			  break;
			}
	      
		} 
   
	} // end for nscan
	due_nmr_read_response(dfd,80);
  
	due_nmr_close_prog(dfd);
}// end main
