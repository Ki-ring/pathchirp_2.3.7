/*
 * pathchirp_rcv.c
 * Copyright (c) 2003 Rice University
 * All Rights Reserved.
 *
 * Permission to use, copy, modify, distribute, and sell this software
 * and its documentation is hereby granted without
 * fee, provided that the above copyright notice appear in all copies
 * and that both that copyright notice and this permission notice
 * appear in supporting documentation, and that the name of Rice
 * University not be used in advertising or publicity pertaining to
 * distribution of the software without specific, written prior
 * permission.  Rice University makes no representations about the
 * suitability of this software for any purpose.  It is provided "as
 * is" without express or implied warranty. 
 *
 * RICE UNIVERSITY DISCLAIMS ALL WARRANTIES WITH REGARD TO THIS
 * SOFTWARE, INCLUDING ALL IMPLIED WARRANTIES OF MERCHANTABILITY AND
 * FITNESS, IN NO EVENT SHALL RICE UNIVERSITY BE LIABLE FOR ANY
 * SPECIAL, INDIRECT OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN
 * AN ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING
 * OUT OF OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS
 * SOFTWARE.
 *
 * Author: Vinay Ribeiro, vinay@rice.edu.  */

/*
 * "pathchirp_rcv" receives chirp packets from "pathchirp_snd", estimates
 *  available bandwidth and writes this to a file.
 *
 *  pathchirp_rcv.c and related code is based on udpread.c of the NetDyn tool.
 *
 */

/*
 * udpread.c
 * Copyright (c) 1991 University of Maryland
 * All Rights Reserved.
 *
 * Permission to use, copy, modify, distribute, and sell this software and its
 * documentation for any purpose is hereby granted without fee, provided that
 * the above copyright notice appear in all copies and that both that
 * copyright notice and this permission notice appear in supporting
 * documentation, and that the name of U.M. not be used in advertising or
 * publicity pertaining to distribution of the software without specific,
 * written prior permission.  U.M. makes no representations about the
 * suitability of this software for any purpose.  It is provided "as is"
 * without express or implied warranty.
 *
 * U.M. DISCLAIMS ALL WARRANTIES WITH REGARD TO THIS SOFTWARE, INCLUDING ALL
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS, IN NO EVENT SHALL U.M.
 * BE LIABLE FOR ANY SPECIAL, INDIRECT OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION
 * OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF OR IN
 * CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 *
 * Author:  Dheeraj Sanghi, Department of Computer Science.
 */


#include "pathchirp_rcv.h"


FILE *fd_instbw;/* file pointers for output files*/
FILE *fd_debug;/* file pointers for debug files*/

FILE *fd_tmp;/*for debugging*/
#ifdef HAVE_SO_TIMESTAMP
struct  msghdr msg;
   struct  iovec iov[1];

   struct cmsghdr *cmptr;
#endif

struct in_addr src_addr;
int fromlen=1;/*must be non-zero, used in recvfrom*/
int debug=0;
int jumbo=1;/*number of pkts per jumbo packet */
u_int32_t request_num=0;/* current request number */
u_int32_t sender_request_num=0;/* current request number */
 u_int32_t chal_no=0;
int cc=0;
int state=0,ack_not_rec_count=0;
struct control_rcv2snd *pkt;/*control packet pointer*/
 char	data_snd[MAXMESG];		/* Maxm size packet */
struct itimerval cancel_time,timeout;/* used in setitimer*/
  struct	sockaddr_in src;	/* socket addr for outgoing packets */
  struct	sockaddr_in dst;	/* socket addr for incoming packets */

int lowcount=0,highcount=0;/*used in sending range updates */

int next_ok_due=0;/* used to send OK packets to sender */

int created_arrays=0;

int max_good_pkt_this_chirp;/* used in chirps affected by coalescence */

u_int32_t cur_num=0; /* current control packet number */

char hostname[MAXHOSTNAMELEN];

char	data[MAXMESG];		/* Maxm size packet */
struct	udprecord *udprecord;	/* log record for a packet */

double total_inst_bw_excursion=0.0;/* sum of chirp estimates over the number of
                            estimates specified */

double perc_bad_chirps=0.0;/*in the last write interval how many
                             chirps were affected by context
                             switching*/
double stop_time;/*time to stop experiment */

/* parameters for excursion detection algorithm */
double decrease_factor=1.5;
int busy_period_thresh=5;


/*context switching threshold */
double context_receive_thresh=0.000010;/*10us*/

int soudp;/*socket for udp connection */


int pkts_per_write;/* how many packet are expected to arrive at each
                      write interval */

int num_inst_bw=11;/* number of estimates to smooth over for mean */

int inst_head=0;/* pointer to current location in circular buffer */

int inst_bw_count=0;/* total number of chirps used in estimation till
                       now*/

double inter_chirp_time;/*time between chirps as stated by the sender */

double chirp_duration;/*transmission time of one chirp */

double write_interval; /* how often to write to file */
int pktsize=1000;

/*rate range in chirp (Mbps)*/
double low_rate=DEFAULT_MIN_RATE,high_rate=DEFAULT_MAX_RATE,avg_rate=DEFAULT_AVG_RATE;


double spread_factor=1.2; /* decrease in spread of packets within the
                             chirp*/

double *qing_delay,*qing_delay_cumsum;
double *rates,*av_bw_per_pkt,*iat;

double *inst_bw_estimates_excursion;/* pointer to interarrivals to look for */

int num_interarrival;/* number of different interarrivals to keep track of */

int first_chirp,last_chirp;/*keeps track of first and last chirp numbers currently in the records*/

int num_pkts_in_info=0;/* how big packet_info currently is*/

int sndPort = SNDPORT;	/* destination UDP port */

int net_option=1; /* network option, Gbps or not */
int chirps_per_write;/*how many chirps will there be per write timer interrupt*/


  char localhost[MAXHOSTNAMELEN];/*string with local host name*/

double min_timer;/*minimum timer granularity*/

struct itimerval wait_time;/* sigalrm time */

struct pkt_info *packet_info;/* keeps track of packet numbers, receive
                                time, send time*/

struct chirprecord *chirp_info;

int no_chirps_recd=0;

in_addr_t gethostaddr(name)
     char *name;
{
   in_addr_t addr;
   register struct	hostent *hp;

   if ((addr = (in_addr_t)inet_addr (name)) != -1)
      return (addr);

   hp = gethostbyname(name);
   if (hp != NULL)
      return (*(in_addr_t *)hp->h_addr);
   else
      return (0);
}



/* usage information */
void usage()
{
  (void) fprintf (stderr,"usage: pathchirp_rcv -S <sender> -t <duration(secs)> <more options, -h prints all options>\n \t -n <number of estimates to smooth over, default=11>\n \t -d <decrease factor (>1), default=1.5>\n \t -b <busy period length (integer >2), default=5>\n \t -U <receiver port (chirp UDP), default=%d (1024-65535)>\n \t -h Help: produces this output\n",SNDPORT);
   (void) fprintf (stderr, "\t -S \t sender host name or IP address \n");
   (void) fprintf (stderr, "\t -J \t number of packets per Jumbo packet,default=1. In case of packet coalescence use values larger than 1, e.g. -J 6 \n");
   (void) fprintf (stderr, "\t -l \t lowest rate (Mbps) to probe at within chirp, default=%fMbps. NOTE: by default pathchirp will find an appropriate probing range.\n",DEFAULT_MIN_RATE);
   (void) fprintf (stderr, "\t -u \t highest rate (Mbps) to probe at within chirp, default=%fMbps\n",DEFAULT_MAX_RATE);
   (void) fprintf (stderr, "\t -p \t packet size <%d-%d>,default=1000 bytes\n",MINPKTSIZE,MAXMESG);
   (void) fprintf (stderr, "\t -t \t duration of experiment(sec), default=600 sec \n");
   (void) fprintf (stderr, "\t -s \t spread factor: ratio of consecutive inter-arrivals within a chirp, default=1.2 \n");
   (void) fprintf (stderr, "\t -a \t average probing rate (Mbps), default=0.3Mbps \n");
   (void) fprintf (stderr, "\t -D print debug information \n");
   (void) fprintf (stderr, "\t -v version\n");

  exit (1);
}


/* close all open files and sockets */  

void close_all()
{
   (void) close (soudp);
   fflush(fd_instbw);
   fclose(fd_instbw);

   if (debug)
     {
       fflush(fd_debug);
       fclose(fd_debug);
     }

}


/* main function executing all others */



int main(argc,argv)
     int	argc;
     char	*argv[];
{

  parse_cmd_line(argc,argv);/*parse options, in parse_cmd_line_rcv.c*/

  create_arrays(); /* allocate memory for different arrays, in alloc_rcv.c */

  lockMe(); /* make sure memory not overwritten, in realtime.c*/

  min_timer=timer_gran();/* find minimum timer granularity */

  /*
    Start the signal handler for SIGALRM.
    The timer is started whenever a packet from a
    new chirp is received.
    Compute available bandwidth after each timer expires.
  */

  Signal(SIGALRM, sig_alrm);/*in signal_alrm_rcv.c*/

 /* contact sender and reply to challenge packet, in control_rcv.c */
 initiate_connection(); 

 /*start receiving chirp packets*/
   receive_chirp_pkts();


 close_all();

  return(0);
}



