#include "pathchirp_rcv.h"

extern void check_reorder_loss();

/* smooths bandwidth estimates and writes to to file */
void write_instant_bw(double inst_av_bw_excursion,double time_stamp)
{
  /*  double inst_median=0.0;*/
  double inst_mean=0.0;

  /*taking latest value and removing old one*/

 total_inst_bw_excursion+=inst_av_bw_excursion;
 total_inst_bw_excursion-=inst_bw_estimates_excursion[inst_head];

 inst_bw_estimates_excursion[inst_head]=inst_av_bw_excursion;

 inst_head++;
 /*circular buffer*/
 if (inst_head>=num_inst_bw)
   inst_head=0;

 if (inst_bw_count>num_inst_bw)
   {

     inst_mean=total_inst_bw_excursion/((double) num_inst_bw);

     inst_mean=inst_mean/1000000.0;/* now in Mbps*/

     /* if estimates are close to either end of chirp probing rates, change
	rates */
     if (inst_mean<1.5*low_rate)
       lowcount++;
     else 
       lowcount=0;
     
     if (inst_mean>0.66*high_rate)
       highcount++;
     else 
       highcount=0;

  /* write to file */
     fprintf(fd_instbw,"%f %f\n",time_stamp,inst_mean);

     fflush(fd_instbw);
   }
 else  inst_bw_count++;

}

/* compute the instantaneous bandwidth during this chirp, new algorithm */

double compute_inst_bw_excursion()
{
  int cur_loc=0;/* current location in chirp */
  int cur_inc=0;/* current location where queuing delay increases */
  int count;
  double inst_bw=0.0,sum_iat=0.0,max_q=0.0;

  /* set all av_bw_per_pkt to zero */
  memset(av_bw_per_pkt, 0, (int)(num_interarrival) * sizeof(double));
    
  /*find first place with an increase in queuing delay*/
  while(qing_delay[cur_inc]>=qing_delay[cur_inc+1] && cur_inc<max_good_pkt_this_chirp)
    cur_inc++;
  
  cur_loc=cur_inc+1; 
  
  /* go through all delays */

  /* Note: in case of interrupt coalescence we consider only 
     that part of the chirp unaffected by it. This should not be
     an issue on low speed links (less than Gigabit). It should also not
be an issue on Gigabit links if you use large packets (say 8KB). */
      if(debug) fprintf(stderr,"\nmax_good_pkt=%d\n",max_good_pkt_this_chirp);
  
  while(cur_loc<=max_good_pkt_this_chirp)
    {
      /* check if queuing delay has decreased to "almost" what it was at cur_inc*/

      if (qing_delay[cur_loc]>NEG_THRESH+1.0)/* marked as negative in case of coalescence, skip this packet*/
	{
	  /* find maximum queuing delay between cur_inc and cur_loc*/
	  
	  if (max_q<(qing_delay[cur_loc]-qing_delay[cur_inc]))
	    max_q=qing_delay[cur_loc]-qing_delay[cur_inc];
	  
	  if (qing_delay[cur_loc]-qing_delay[cur_inc]<(max_q/decrease_factor))
	    {
	      if (cur_loc-cur_inc>=busy_period_thresh)
		{
		  for (count=cur_inc;count<=cur_loc-1;count++)
		    {
		      /*mark all increase regions between cur_inc and cur_loc as having
			available bandwidth equal to their rates*/
		      if (qing_delay[count]<qing_delay[count+1])
			av_bw_per_pkt[count]=rates[count];
		    }
		}
	      /* find next increase point */
	      cur_inc=cur_loc;
	      while(qing_delay[cur_inc]>=qing_delay[cur_inc+1] && cur_inc<max_good_pkt_this_chirp)
		cur_inc++;
	      cur_loc=cur_inc;
	      /* reset max_q*/
	      max_q=0.0;
	    }
	}
      cur_loc++;      
    }

  if(cur_inc==max_good_pkt_this_chirp)
    cur_inc--;
  
    /*mark the available bandwidth during the last excursion as the rate at its beginning*/

  for(count=cur_inc;count<max_good_pkt_this_chirp;count++)
    {
      av_bw_per_pkt[count]=rates[cur_inc];
    }
  
  
  /* all unmarked locations are assumed to have available 
     bandwidth described by the last excursion */
  
  for(count=0;count<max_good_pkt_this_chirp;count++)
    {
      if (av_bw_per_pkt[count]<1.0)
	av_bw_per_pkt[count]=rates[cur_inc];
      sum_iat+=iat[count];
      inst_bw+=av_bw_per_pkt[count]*iat[count];
    }

  inst_bw=inst_bw/sum_iat;
  return(inst_bw);
  
}


/* compute queuing delay, packet drops, and available bandwidth estimates*/
void compute_stats()
{ 
  int count;
  double total_qing_delay=0.0;
  double inst_bw_excursion=0.0;
  long nc,np;
  int prev_pkt=0,num_bad_pkts_this_chirp=0,num_bad_chirps=0,context_switch=0;
  int num_consec_bad=0;/* number of consecutive chirp interarrivals affected by 
			  interrupt coalescence and context switching */

  if (debug)
    fprintf(stderr,"\nCompute Stats\n");

  check_reorder_loss();
  
  /*fprintf(stderr,"firstchirp=%d,last chirp=%d\n",first_chirp,last_chirp);*/
  
  for (count=0;count<num_pkts_in_info;count++)
    {
      nc=packet_info[count].chirp_num;
      np=packet_info[count].num;

      /*skip packet if chirp_info overflow*/
      if ((nc-first_chirp)>=(2*chirps_per_write))
        {
          continue;
        }

      /*if packet reorder */
      if (chirp_info[nc-first_chirp].reorder==2 || packet_info[count].context_switch==2)
	continue;
      else /*no reorder*/
	{
	  /* if loss */
	  if (chirp_info[nc-first_chirp].num<num_interarrival+1)
	    {

	      /*if more than 1 chirp*/
	      if (last_chirp>first_chirp)
		{
		  if (nc>first_chirp && nc<last_chirp && np==chirp_info[nc-first_chirp].last_pkt)/*if not first or last chirp*/
		    num_bad_chirps++;
		}
	      else
		{
		  if (np==chirp_info[nc-first_chirp].last_pkt)
		    write_instant_bw(low_rate*1000000.0/2.0,packet_info[count].rcv_time);
		}
	       continue;
	    }
	  else /* if no loss  */
	    {
	      qing_delay[np-1]=packet_info[count].rcv_time-packet_info[chirp_info[nc-first_chirp].first_pkt_loc].rcv_time-packet_info[count].snd_time+packet_info[chirp_info[nc-first_chirp].first_pkt_loc].snd_time;

	      /*context switching/interrupt coalescence*/
	      if (np>1)
		{

		  /* used only for jumbo packets*/
		  if (packet_info[count].good_jumbo_pkt==0 && max_good_pkt_this_chirp==num_interarrival)
		    {
		      max_good_pkt_this_chirp=np-1;
		    }

		  if(packet_info[count].rcv_time-packet_info[prev_pkt].rcv_time<context_receive_thresh)
		    {
		      /*		      fprintf(stderr,"np=%ld,time diff=%f\n",np,packet_info[count].rcv_time-packet_info[prev_pkt].rcv_time);*/
		      num_consec_bad++;

		      qing_delay[np-1]=NEG_THRESH;/*marked as negative*/

		      /* if CONSEC_BAD_PKTS consecutive interarrivals are 
			 influenced by coalescence */
		      if (num_consec_bad>=CONSEC_BAD_PKTS && max_good_pkt_this_chirp==num_interarrival)
			{
			  max_good_pkt_this_chirp=np-CONSEC_BAD_PKTS-1;
			  if (debug) fprintf(stderr,"max good pkt=%d\n",max_good_pkt_this_chirp);
			  if (max_good_pkt_this_chirp<2) 
			    max_good_pkt_this_chirp=2;
			}
		    }
		  else {
		    num_consec_bad=0;
		  }
		}
	      else
		{
		    max_good_pkt_this_chirp=num_interarrival;
		}

	      prev_pkt=count;/*location of previous packet in this "good" chirp*/

	      total_qing_delay+=qing_delay[np-1];

	      /* if last chirp packet then compute available bandwidth */
	      if (np==(num_interarrival+1))
		{
		  if(context_switch==1)
		    {
		      fprintf(stderr,"\rbad chirp, number=%ld\n",nc);
		      continue;
		    }
		  else
		    {
		      inst_bw_excursion=compute_inst_bw_excursion();
		      write_instant_bw(inst_bw_excursion,packet_info[count].rcv_time);
		    }
		  total_qing_delay=0.0;
		  num_bad_pkts_this_chirp=0;
		}
		
	    }
	      
	    
	}
    }
}

/* if estimates too low or too high update probing range */
int check_for_new_pars()
{
  int resetflag=0;

  if (lowcount>2)
    {

      if (low_rate<=1.0) /*less than 1Mbps, have smaller range of rates, that is fewer packets per chirp. This allows us to send more chirps per unit time.*/
	{
	  high_rate=low_rate*4;
	  low_rate=low_rate/4;
	}
      else{
	  high_rate=low_rate*7;
	  low_rate=low_rate/7;
	}

      resetflag=1;
      lowcount=0;
    }

  if (highcount>2 && high_rate<0.9*(double)MAX_HIGH_RATE)
    {

      if (high_rate>1.0) 
	{
	  low_rate=high_rate/7;
	  high_rate=high_rate*7;
	}
      else{
	  low_rate=high_rate/4;
	  high_rate=high_rate*4;
	}
      resetflag=1;
      highcount=0;
    }
  else 
    if (highcount>2)
      highcount=0;

  if (resetflag)
    {
      /* make sure average rate is not too high  */
      if (avg_rate>low_rate/5)
	avg_rate=low_rate/5;
      else
	if (avg_rate<low_rate/10)
	  {
	    avg_rate=low_rate/10;
	  }
      /*make sure avg_rate is less than MAX_AVG_RATE*/
      if(avg_rate>MAX_AVG_RATE)
	avg_rate=MAX_AVG_RATE;

      /*make sure avg_rate is more than MIN_AVG_RATE*/

      if(avg_rate<MIN_AVG_RATE)
	avg_rate=MIN_AVG_RATE;

      compute_parameters();
      update_rates_iat();
    }

  return(resetflag);

}
