#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "due-nmr-lib.h"

// mt-cpmg3.c array param is frequency offset
#define NPTS 40000
#define NECHO 40

//#define NVAL 2
//double offset1[NVAL] = {8000,-8000};
//double offset2[NVAL] = {8000,-8000};


#define NVAL 5
double offset1[NVAL] = {8000,-8000,8000,-16000,16000};
double offset2[NVAL] = {8000,-8000,-8000,-16000,-16000};

//double offset1[NVAL] = {};
//double offset2[NVAL] = {};
//#define NVAL 30
//double offset1[NVAL] = {-15000,-13500,-12000,-10500,-9000,-7500,-6000,-4500,-3000,-1500, 1500, 3000, 4500, 6000, 7500, 9000, 10500, 12000, 13500, 15000,-15000,-13500,-12000,-10500,-9000,-7500,-6000,-4500,-3000,-1500};
//double offset2[NVAL] = {-15000,-13500,-12000,-10500,-9000,-7500,-6000,-4500,-3000,-1500, 1500, 3000, 4500, 6000, 7500, 9000, 10500, 12000, 13500, 15000,15000,13500,12000,10500,9000,7500,6000,4500,3000,1500};
//#define NVAL 12
//double offset1[NVAL] = {-17500,-20000,-22500,-25000,25000,22500,20000,17500,17500,20000, 22500, 25000 };
//double offset2[NVAL] = {-17500,-20000,-22500,-25000,25000,22500,20000,17500,-17500,-20000, -22500, -25000};
//#define NVAL 43
//double offset1[NVAL] = {-15000,-13500,-12000,-10500,-9000,-7500,-6000,-4500,-3000,-1500, 1500, 3000, 4500, 6000, 7500, 9000, 10500, 12000, 13500, 15000,-15000,-13500,-12000,-10500,-9000,-7500,-6000,-4500,-3000,-1500,-17500,-20000,-22500,-25000,25000,22500,20000,17500,17500,20000, 22500, 25000};
//double offset2[NVAL] = {-15000,-13500,-12000,-10500,-9000,-7500,-6000,-4500,-3000,-1500, 1500, 3000, 4500, 6000, 7500, 9000, 10500, 12000, 13500, 15000,15000,13500,12000,10500,9000,7500,6000,4500,3000,1500,-17500,-20000,-22500,-25000,25000,22500,20000,17500,-17500,-20000, -22500, -25000};
//#define NVAL 1
//double offset1[1],offset2[1];

char basename[11]="junk";

double recovery = 200e-6;
double mt_time = 1.;
int32_t accum[NVAL][NPTS*2];

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

  

  int i,j,val,k;
  int16_t data[NPTS*2];
  char flags,rval;
  FILE *outfile;
  int dummy = 0, nscan, npts;
  double freq = 63600;
  
  if (argc != 4){
    printf("usage: ihmt nscan pw90 pw180 (all integers, pw's in us\n");
    return 0;
  }

  nscan = atoi(argv[1]);
  pw90 = atoi(argv[2])*1e-6;
  pw180 = atoi(argv[3])*1e-6;
  printf("doing: nscan = %i\npw90 = %g\npw180 = %g\n",nscan,pw90,pw180);

  
  due_nmr_init_program(&prog);
  dfd = due_nmr_open_prog("/dev/cu.usbmodem142301");
  
  memset(accum,0,NPTS*2*4*NVAL);
  printf("open program, fd is: %i\n",dfd);
  due_nmr_set_freq(dfd, freq);
  
  due_nmr_read_response(dfd,2);

  for(i=0;i<nscan+dummy;i++) {
    for (val=0;val<NVAL;val++){
      //////////////////
      due_nmr_init_program(&prog);
      // add some events
      // prog, outputs, tx, phase, rx, duration, phase_reset
      // tx relay control is D33 - value position 2
      // rx relay contrl is D34 - value position 4
      //    pw90= 150e-6; // at full power 90 might be 90us.
      due_nmr_add_event(&prog, 0, 0, 0., 0,.1,0);
      
      due_nmr_add_event(&prog, 2, 0, 0., 0, .002, 0);  // flip tx relay on. Don't Do phase reset.
      if (val == 42)
      	due_nmr_add_new_freq(&prog, 2, 0, 0., 0, 700*1e-3, freq); // dummy - no prepulse
            else{
	for(k=0;k<70;k++){
	  due_nmr_add_new_freq(&prog, 2, 1, 0., 0, 2e-3, freq+offset1[val]); // the MT pulse
	  due_nmr_add_event(&prog, 2, 0, 0., 0, 1e-3 , 0); // inter pulse delay
	  
	  due_nmr_add_new_freq(&prog, 2, 1, 0., 0, 2e-3, freq+offset2[val]); // the MT pulse
	  due_nmr_add_event(&prog, 2, 0, 0., 0, 5e-3, 0);  // inter pulse delay
	}
      }
      due_nmr_add_new_freq(&prog, 2, 0, 0., 0, recovery, freq); // recovery delay, reset freq
      
      due_nmr_add_event(&prog, 2, 1, 180*(i%2), 0, pw90, 0);  // the pulse
      //due_nmr_add_event(&prog, 2, 0, 0., 0, .016666-pw180/2-.002, 0);  // echo
      due_nmr_add_event(&prog, 2, 0, 0., 0, .016-pw180/2-.002, 0);  // echo
      for(j=0;j<NECHO;j++){
	due_nmr_add_event(&prog, 2, 0, 0., 0, .002, 0);  // flip tx relay on.
	due_nmr_add_event(&prog, 2, 1, 90, 0, pw180, 0);  // the pulse
	due_nmr_add_event(&prog, 2, 0, 0., 0, .001, 0);  // allow some ring down
	due_nmr_add_event(&prog, 0, 0, 0., 0, .001, 0);  // neither
	due_nmr_add_event(&prog, 4, 0, 0., 0, .001, 0);  // rx relay on.
	//      due_nmr_add_event(&prog, 4, 0, 0., 1, .033332-pw180-.005, 0); // collect _
	due_nmr_add_event(&prog, 4, 0, 0., 1, .032-pw180-.005, 0); // collect _33ms data = 800 poin
      }
      
      //    npts = (0.033332-pw180-.005)/50e-6;
      npts = (0.032-pw180-.005)/50e-6;
      npts = npts*NECHO;
      due_nmr_last_event(&prog, 0, 0, 0., 0, 5);
      
      due_nmr_finalize_program(&prog);
      if (i== 0)
	due_nmr_dump_program(&prog);
      /////////////////
      
      //       due_nmr_set_freq(dfd, 65700.);
      //read_response(dfd,2);
      
      printf("\nCalling download, for i val: %i\n",i);
      due_nmr_download_prog(dfd, &prog);
      if(i== 0 && val == 0)
	due_nmr_run_program(dfd, 'Y');
      // should check status to make sure its running.
      rval = due_nmr_read_data(dfd, npts,120, data, &flags);
      printf("return from read, got flags: %i\n",flags&0xff);
      if (i>dummy-1){
	sprintf(name,"%s-%i",basename,val);
	outfile = fopen(name,"w");
	fprintf(outfile,"# ihmt pw: %f %f offsets: %f,%f, scans complete: %i\n",pw90,pw180,offset1[val],offset2[val],i+1);
	for (j=0;j<npts;j++){
	  accum[val][2*j] += ((i%2)*2-1)*data[2*j];
	  accum[val][2*j+1] += ((i%2)*2-1)*data[2*j+1];
	  fprintf(outfile,"%i %i\n",accum[val][2*j],accum[val][2*j+1]);
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
  }
  due_nmr_read_response(dfd,80);
  
  due_nmr_close_prog(dfd);
}
