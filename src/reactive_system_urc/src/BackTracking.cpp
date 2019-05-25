#include "Backtracking.h"

/*
  read current path msg

  based on response time (time between consecutive reads) associate it with a
  connectivity status (good if response time <5 sec, medium if between 5 and 10
  sec, poor if longer than 10 sec)

  create reverse path to previous position with good connectivity while
  connectivity is not poor

  when connectivity is poor then publish the reverse path
*/
