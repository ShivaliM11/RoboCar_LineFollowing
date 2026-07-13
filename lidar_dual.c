// ==== DUAL LIDAR (L=ttyACM1, R=ttyACM0), alternates active on each turn ====
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <pthread.h>

#define LX_WIDTH 8
#define LX_HEIGHT 8
#define LX_THRESHOLD 127
#define LX_GAP_MIN 3
#define LX_DOUBLE_VOTES 3
#define LX_STABLE 3
#define LX_SINGLE_HOLD 50

struct lx_row { unsigned int pixel[LX_WIDTH]; };
struct lx_frame { struct lx_row row[LX_HEIGHT]; };

// R starts active; ir_thread flips this at each turn
volatile int  g_active_sensor = 0;          // 0 = R, 1 = L
volatile bool g_lidar_double_stop = false;  // motor thread reads this
pthread_mutex_t g_lx_lock = PTHREAD_MUTEX_INITIALIZER;

int  g_count_empty = 0, g_count_single = 0, g_count_double = 0;

static int lx_classify_row(struct lx_row *row)
{
  bool seen=false, gap_ok=false, any=false; int gap=0;
  for (int c=0;c<LX_WIDTH;c++){
    bool d = (row->pixel[c] <= LX_THRESHOLD);
    if(d){ any=true; if(seen&&gap_ok) return 2; seen=true; gap=0; }
    else if(seen){ gap++; if(gap>=LX_GAP_MIN) gap_ok=true; }
  }
  return any?1:0;
}
static int lx_classify(struct lx_frame *f)
{
  int v[3]={0,0,0};
  for(int r=0;r<LX_HEIGHT-1;r++) v[lx_classify_row(&f->row[r])]++;
  if(v[2]>=LX_DOUBLE_VOTES) return 2;
  if(v[1]+v[2]>0) return 1;
  return 0;
}
static void lx_getline(FILE*f,char*b,size_t n){
  char*l=NULL;size_t z=0;ssize_t g;b[0]=0;
  if(!f)return; g=getline(&l,&z,f);
  if(g>0&&l&&(size_t)g<n) strcpy(b,l); free(l);
}

struct lx_param { FILE *serial; int which; };  // which: 0=R,1=L

void *lidar_thread(void *arg)
{
  struct lx_param *p = (struct lx_param*)arg;
  char line[1024];
  struct lx_frame fr; memset(&fr,0,sizeof(fr));
  int confirmed=-1, cand=-1, cand_n=0, hold=0; bool holding=false;

  while(1){
    lx_getline(p->serial, line, sizeof(line));
    if(!(line[0]=='y'&&line[1]>='0'&&line[1]<='7'&&strlen(line)>3)) continue;
    int r=line[1]-'0';
    sscanf(&line[3],"%u,%u,%u,%u,%u,%u,%u,%u",
      &fr.row[r].pixel[0],&fr.row[r].pixel[1],&fr.row[r].pixel[2],&fr.row[r].pixel[3],
      &fr.row[r].pixel[4],&fr.row[r].pixel[5],&fr.row[r].pixel[6],&fr.row[r].pixel[7]);
    if(r!=LX_HEIGHT-1) continue;

    // only the active sensor counts / stops
    if(p->which != g_active_sensor){ confirmed=-1; cand=-1; cand_n=0; holding=false; continue; }

    int st=lx_classify(&fr);
    if(st==cand) cand_n++; else { cand=st; cand_n=1; }

    if(cand_n>=LX_STABLE && cand!=confirmed){
      if(cand==1){ holding=true; hold=0; confirmed=1; }
      else if(cand==2){
        holding=false; confirmed=2;
        pthread_mutex_lock(&g_lx_lock);
        g_count_double++; g_lidar_double_stop=true;
        pthread_mutex_unlock(&g_lx_lock);
      } else {
        if(holding){ holding=false; pthread_mutex_lock(&g_lx_lock); g_count_single++; pthread_mutex_unlock(&g_lx_lock); }
        confirmed=0;
        pthread_mutex_lock(&g_lx_lock);
        g_count_empty++; g_lidar_double_stop=false;
        pthread_mutex_unlock(&g_lx_lock);
      }
    }
    if(holding){ hold++; if(hold>=LX_SINGLE_HOLD){ holding=false; pthread_mutex_lock(&g_lx_lock); g_count_single++; pthread_mutex_unlock(&g_lx_lock);} }
  }
  return NULL;
}
