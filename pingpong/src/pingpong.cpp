#include <ros/ros.h>
#include <pingpong/PingPong.h>
#include <pingpong/Status.h>
#include <err.h>


static size_t const default_data_size(128); 
static double const default_update_rate(10.0);
static size_t const default_queue_size(1);

static bool pong_mode(false);
static bool tcp_mode(false);
static size_t data_size(default_data_size);
static double update_rate(default_update_rate);
static size_t queue_size(default_queue_size);

static ros::Publisher pingpong_pub;
static ros::Publisher status_pub;
static ros::Subscriber pingpong_sub;
static size_t counter;
static std::vector<uint8_t> data;
static size_t nrcv;
static size_t prevseq;
static size_t ds;
static size_t dsmin;
static size_t dsmax;
static size_t dssum;
static double dsmean;


static void usage(bool ok)
{
  FILE * fs(stderr);
  if (ok) {
    fs = stdout;
  }
  
  fprintf(fs,
	  "pingpong usage:\n"
	  "\t-h\t\thelp message\n"
	  "\t-o\t\toperate in pong mode instead of ping mode\n"
	  "\t-t\t\tuse TCP (reliable) instead of UDP (unreliable) transport\n"
	  "\t-n\tbytes\tsize of message payload (optional suffix k or m)\n"
	  "\t-r\trate\trate of update timer in Hz\n"
	  "\t-b\tbacklog\tnumber of messages to keep in queues\n"
	  "defaults:\n"
	  "  data size %zu bytes\n"
	  "  update rate %f Hz\n"
	  "  backlog %zu messages\n",
	  default_data_size, default_update_rate, default_queue_size);
  
  exit(ok ? EXIT_SUCCESS : EXIT_FAILURE);
}


static void update(ros::TimerEvent const & timer_event)
{
  if (pong_mode) {
    /* nop */
  }
  else {
    pingpong::PingPong pmsg;
    pmsg.seq = counter;
    pmsg.data = data;
    pingpong_pub.publish(pmsg);
  }
  
  if (0 < nrcv) {
    pingpong::Status smsg;
    smsg.ds = ds;
    smsg.dsmin = dsmin;
    smsg.dsmax = dsmax;
    smsg.dsmean = dsmean;
    status_pub.publish(smsg);
  }
  
  ++counter;
}


static void msg_cb(boost::shared_ptr<pingpong::PingPong const> const & msg)
{
  if (pong_mode) {
    pingpong::PingPong pmsg;
    pmsg.seq = msg->seq;
    pmsg.data = data;
    pingpong_pub.publish(pmsg);
    
    prevseq = msg->seq;
    if (0 == nrcv) {
      ds = 1;
    }
    else {
      ds = msg->seq - prevseq;
    }
  }
  else {
    ds = counter - msg->seq;
  }
  
  if (0 == nrcv) {
    dsmin = ds;
    dsmax = ds;
    dssum = ds;
    dsmean = ds;
  }
  else {
    dssum += ds;
    if (ds < dsmin) {
      dsmin = ds;
    }
    if (ds > dsmax) {
      dsmax = ds;
    }
    dsmean = dssum * 1.0 / nrcv;
  }
  
  ++nrcv;
}


int main(int argc, char** argv)
{
  {
    std::ostringstream node_name;
    node_name << "pingpong" << getpid();
    ros::init(argc, argv, node_name.str());
  }
  
  for (int ii(1); ii < argc; ++ii) {
    std::string const arg(argv[ii]);
    if ("-h" == arg) {
      usage(true);
    }
    else if ("-o" == arg) {
      pong_mode = true;
    }
    else if ("-t" == arg) {
      tcp_mode = true;
    }
    else if ("-n" == arg) {
      ++ii;
      if (ii >= argc) {
	warnx("missing argument");
	usage(false);
      }
      char unit('\0');
      int const nn(sscanf(argv[ii], "%zu%[km]", &data_size, &unit));
      if (0 == nn) {
	warnx("invalid argument");
	usage(false);
      }
      if ((1 == nn) && '\0' != unit) {
	warnx("missing number");
	usage(false);
      }
      switch (unit) {
      case 'm':
	data_size *= 1024;
      case 'k':
	data_size *= 1024;
      }	// no need for break or default
    }
    else if ("-r" == arg) {
      ++ii;
      if (ii >= argc) {
	warnx("missing argument");
	usage(false);
      }
      if (1 != sscanf(argv[ii], "%lf", &update_rate)) {
	warnx("invalid argument");
	usage(false);
      }
    }
    else if ("-b" == arg) {
      ++ii;
      if (ii >= argc) {
	warnx("missing argument");
	usage(false);
      }
      if (1 != sscanf(argv[ii], "%zu", &queue_size)) {
	warnx("invalid argument");
	usage(false);
      }
    }
    else {
      warnx("invalid option");
      usage(false);
    }
  }
  
  ros::NodeHandle nn;
  ros::TransportHints tp;
  if (tcp_mode) {
    tp.reliable();
  }
  else {
    tp.unreliable();
  }
  
  if (pong_mode) {
    pingpong_pub = nn.advertise<pingpong::PingPong>("/pong", queue_size);
    pingpong_sub = nn.subscribe<pingpong::PingPong>("/ping", queue_size,
						    msg_cb, tp);
    status_pub = nn.advertise<pingpong::Status>("/pongstat", queue_size);
  }
  else {
    pingpong_pub = nn.advertise<pingpong::PingPong>("/ping", queue_size);
    pingpong_sub = nn.subscribe<pingpong::PingPong>("/pong", queue_size,
						    msg_cb, tp);
    status_pub = nn.advertise<pingpong::Status>("/pingstat", queue_size);
  }
  
  counter = 0;
  nrcv = 0;
  data.resize(data_size);
  for (size_t ii(0); ii < data_size; ++ii) {
    data[ii] = 0xff & (ii % 256);
  }
  
  ros::Timer timer(nn.createTimer(ros::Duration(1 / update_rate), update));
  ros::spin();
}
