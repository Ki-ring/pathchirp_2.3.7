#include "pathchirp_rcv.h"


/* get command line parameters */
void parse_cmd_line(argc,argv)
  int	argc;
     char	*argv[];
{
  char	*ptr;			/* to traverse the arguments */
  double duration=600.0;
  struct timeval tp_start;

      argc--; argv++;

  /* go through the arguments */
   while (argc > 0) {
      ptr = *argv;
      while (*ptr) switch (*ptr++) {
 case 'l':/* lowest rate to probe at */
       if (*ptr == 0) {
	 argc--; argv++;
	 if (*argv == 0) {
	   (void) fprintf (stderr,
			   "pathchirp_rcv: no lowest rate '-l'.\n");
	   (void) exit (1);
	 }
	 low_rate = atof (*argv);
       }
       else {
	 low_rate = atof (*argv);
	 *ptr = 0;
       }
       fprintf(stderr,"low rate is %f \n",low_rate);
       break;
       
     case 'u':/* highest rate to probe at */
       if (*ptr == 0) {
	 argc--; argv++;
	 if (*argv == 0) {
	   (void) fprintf (stderr,
			   "pathchirp_rcv: no highest rate '-u'.\n");
	   (void) exit (1);
	 }
	 high_rate = atof (*argv);
       }
       else {
	 high_rate = atof (*argv);
	 *ptr = 0;
       }
       fprintf(stderr,"high rate is %f \n",high_rate);
       break;
       
       
     case 'a':
       if (*ptr == 0) {
	 argc--; argv++;
	 if (*argv == 0) {
	   (void) fprintf (stderr,
			   "pathchirp_rcv: no average rate '-a'.\n");
	   (void) exit (1);
	 }
	 avg_rate = atof (*argv);
       }
       else {
	 avg_rate = atof (*argv);
	 *ptr = 0;
       }
       fprintf(stderr,"avg_rate is %f \n",avg_rate);

       break;
       
     case 't':
       if (*ptr == 0) {
	      argc--; argv++;
	      if (*argv == 0) {
		(void) fprintf (stderr,
				"pathchirp_rcv: no duration '-t'.\n");
		(void) exit (1);
	      }
	      duration = atof (*argv);
       }
       else {
	 duration = atof (*argv);
	 *ptr = 0;
       }

       break;
       
       
     case 's':
       if (*ptr == 0) {
	 argc--; argv++;
	 if (*argv == 0) {
	   (void) fprintf (stderr,
			   "pathchirp_rcv: no spread_factor '-t'.\n");
	   (void) exit (1);
	 }
	 spread_factor = atof (*argv);
       }
       else {
	 spread_factor = atof (*argv);
	 *ptr = 0;
       }
       fprintf(stderr,"spread  is %f \n",spread_factor);

       break;
       
       
     case '-':
       break;


        case 'p':		/* packet size */
       if (*ptr == 0) {
	 argc--; argv++;
	 if (*argv == 0) {
	   (void) fprintf (stderr,
			   "pathchirp_rcv: no packet size given with '-p'.\n");
	   (void) exit (1);
	 }
	 pktsize = atoi (*argv);
       }
       else {
	 pktsize = atoi (*argv);
	 *ptr = 0;
       }
       break;
       

        case 'J':		/* Jumbo size in packets*/
       if (*ptr == 0) {
	 argc--; argv++;
	 if (*argv == 0) {
	   (void) fprintf (stderr,
			   "pathchirp_rcv: no Jumbo size given with '-J'.\n");
	   (void) exit (1);
	 }
	 jumbo = atoi (*argv);
       }
       else {
	 jumbo = atoi (*argv);
	 *ptr = 0;
       }
       break;
       


     case 'S':
       if (*ptr == 0) {
	 argc--; argv++;
	 if (*argv == 0) {
	   (void) fprintf (stderr,
			   "pathchirp_rcv: no destination host given with '-S'.\n");
	   (void) exit (1);
	 }
	 (void) strcpy (hostname, *argv);
       }
       else {
	 (void) strcpy (hostname, ptr);
	 *ptr = 0;
       }
       fprintf(stderr,"Sender host is:  %s \n",hostname);
       break;

      case 'D':
	debug = 1;
	break;

      case 'v':
	fprintf(stderr,"pathChirp version %s\n",VERSION);
	exit(0);
	break;
              
     case 'b':/* busy period length */
       if (*ptr == 0) {
	 argc--; argv++;
	 if (*argv == 0) {
	   (void) fprintf (stderr,
			   "pathchirp_rcv: no busy period length '-b'.\n");
	   (void) exit (1);
	 }
	 busy_period_thresh = atoi (*argv);
       }
       else {
	 busy_period_thresh = atoi (*argv);
	 *ptr = 0;
       }
       fprintf(stderr,"busy period length is %d \n",busy_period_thresh);
       break;

     case 'd':/* decrease factor */
       if (*ptr == 0) {
	 argc--; argv++;
	 if (*argv == 0) {
	   (void) fprintf (stderr,
			   "pathchirp_rcv: no decrease factor '-d'.\n");
	   (void) exit (1);
	 }
	 decrease_factor = atof (*argv);
       }
       else {
	 decrease_factor = atof (*argv);
	 *ptr = 0;
       }
       fprintf(stderr,"decrease factor is %f \n",decrease_factor);
       break;

     case 'n':/* number of inst. bw. estimates to smooth over*/
       if (*ptr == 0) {
	 argc--; argv++;
	 if (*argv == 0) {
	   (void) fprintf (stderr,
			   "pathchirp_rcv: no averaging length '-n'.\n");
	   (void) exit (1);
	 }
	 num_inst_bw = atoi (*argv);
       }
       else {
	 num_inst_bw = atoi (*argv);
	 *ptr = 0;
       }

       break;
           
 case 'U':		/* UDP port */
       if( *ptr == 0) {
	 argc--; argv++;
	 if (*argv == 0) {
	   (void) fprintf (stderr,
			   "pathchirp_rcv: no port number given with '-U'.\n");
	   (void) exit (1);
	 }
	 sndPort = atoi (*argv);
       }
       else {
	 sndPort = atoi (ptr);
	 *ptr = 0;
       }
       break;
       
 
      case 'h':
      case 'H':
	 usage();
       
      default:
	 (void) fprintf (stderr,
			 "pathchirp_rcv: Unknown option '%c'\n", ptr[-1]);

      }
      argc--; argv++;
   }
 /* checking  parameters */


   /* set the destination as far as sender is concerned
    * i.e. echo host or the final destination host */
     src_addr.s_addr = gethostaddr(hostname);
   if (src_addr.s_addr == 0) {
      (void) fprintf (stderr, "pathchirp_rcv: %s: unknown host\n",
		      hostname);
     usage();
      (void) exit (1);
   }
   else{
     /* initializing */
       bzero((char *)&src, sizeof (src));

     src.sin_addr = src_addr;

   }

  if (decrease_factor <=1.0)
     {perror ("pathchirp_rcv: decrease_factor invalid");
	 (void) exit (1);
     }

  if (duration <0)
     {perror ("pathchirp_rcv: time duration invalid");
	 (void) exit (1);
     }
  else
    {
      (void) gettimeofday (&tp_start, (struct timezone *) 0);
      stop_time=(double)tp_start.tv_sec+(((double)tp_start.tv_usec)/1000000.0)+duration;
    }

  if (jumbo<1 || jumbo>20)
     {perror ("pathchirp_rcv: jumbo must be between 1 and 20");
	 (void) exit (1);
     }

  if (busy_period_thresh <=2)
     {perror ("pathchirp_rcv: busy_period_thresh invalid, must be integer >2");
	 (void) exit (1);
     }

  if (sndPort<1024 ||  sndPort>65535)
     {perror ("pathchirp_rcv: Port number must be in [1024-65535]");
	 (void) exit (1);
     }

 /*check if parameters ok*/
  if (pktsize<MINPKTSIZE)
    {
      fprintf(stderr,"pathchirp_rcv: packet size too small, using minimum sized %d byte packet\n",MINPKTSIZE);
      pktsize=MINPKTSIZE;
    }
  else
    {
      if (pktsize>MAXMESG)
	{
	  fprintf(stderr,"pathchirp_rcv: packet size too big, using maximum sized %d byte packet\n",MAXMESG);
	  pktsize=MAXMESG;
	}
    }

  if(spread_factor<MINSPREAD)
    fprintf(stderr,"pathchirp_rcv: packet spread too small, using minimum packet spread %f\n",MINSPREAD); 

  if( low_rate<0.0 || high_rate<0.0 || avg_rate<0.0 || low_rate>high_rate)
    {
      perror("pathchirp_rcv: probing rates invalid\n"); 
      exit(0);
    }


 if (avg_rate>MAX_AVG_RATE)
    {
      fprintf(stderr,"Average rate too high, reducing to %f Mbps\n",MAX_AVG_RATE);
      avg_rate=MAX_AVG_RATE;

    }

  if (avg_rate<MIN_AVG_RATE)
    {
      fprintf(stderr,"Average rate too low, increasing to %f Mbps\n",MIN_AVG_RATE);
      avg_rate=MIN_AVG_RATE;

    }
  if (high_rate>2000.0*low_rate)
    {
      fprintf(stderr,"Ratio of high/low rate very large, increasing low_rate\n");
      low_rate=high_rate/1000.0;

    }
  if (high_rate<5.0*low_rate)
    {
      fprintf(stderr,"Ratio of high/low rate very low, increasing high_rate\n");
      high_rate=5*low_rate;

    }

  /* opening files */
  gethostname(localhost,sizeof(localhost));
  open_dump_files(hostname,localhost);

  compute_parameters();

}


/* computes number of packets, chirp duration and inter-chirp time */
void compute_parameters()
{
  double interarrival;
  double new_high_rate;/*used just in case given rates lead to an extra large chirp*/


  if (high_rate>MAX_HIGH_RATE)
    {
      high_rate=MAX_HIGH_RATE;
      if (low_rate>(double)MAX_HIGH_RATE/10.0)
	low_rate=(double)MAX_HIGH_RATE/10.0;
    }

  new_high_rate=low_rate;


  interarrival=((double)pktsize)*8/(1000000.0*low_rate);
  chirp_duration=0;
  
  num_interarrival=0;
  while(interarrival>((double)pktsize)*8/(1000000.0*high_rate))
    {
      num_interarrival++;
      chirp_duration+=interarrival*(double)jumbo;
      interarrival=interarrival/spread_factor;

      new_high_rate=new_high_rate*spread_factor;
      if (num_interarrival>=MAXCHIRPSIZE-1)
	{
	  if(debug) fprintf(stderr,"Chirp too big, using MAXCHIRPSIZE, reducing high_rate\n");
	  high_rate=new_high_rate;
	break;
	}
     }

  fprintf(stderr,"\nUpdating probing range:low=%f,high=%fMbps\n",low_rate,high_rate);

  max_good_pkt_this_chirp=num_interarrival;/*initializing*/

  inter_chirp_time=((double)(num_interarrival+1))*8.0*jumbo*pktsize/(avg_rate*1000000.0);
  if (inter_chirp_time<=chirp_duration)
    inter_chirp_time=2.0*chirp_duration;

  /*write once every chirp */
  write_interval=chirp_duration+(inter_chirp_time-chirp_duration)/2;

  /* if timer granularity too large change write_interval*/
  if (write_interval<min_timer)
    {
      if (debug) fprintf(stderr,"write int=min timer\n");
    write_interval=min_timer;
    }
  /*allocating enough memory to store packet information if not enough*/
  
  if((int)(1+(write_interval/(inter_chirp_time)))>chirps_per_write && created_arrays)
    {
      chirps_per_write=(int)(1+(write_interval/(inter_chirp_time)));
      pkts_per_write=(int)((MAXCHIRPSIZE-1+1)*chirps_per_write);
      
      fprintf(stderr,"chirps_per_write=%d, pkts_per_write=%d\n",chirps_per_write,pkts_per_write);
       
      realloc(packet_info,pkts_per_write*2*sizeof(struct pkt_info));      
      realloc(chirp_info,chirps_per_write*2*sizeof(struct chirprecord));
      
    }
  return;
  
}
