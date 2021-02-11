#include <algorithm>
#include <cassert>
#include <cctype>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <unordered_map>
#include <unordered_set>
#include <memory>
#include <string>
#include <vector>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <atomic>
#include <boost/asio.hpp>
#include <boost/thread/thread_pool.hpp>
#include <gperftools/profiler.h>

#include <google/protobuf/io/zero_copy_stream_impl.h>

#include <boost/filesystem.hpp>
#include "rgd.pb.h"

#include "rgd_op.h"
#include "z3solver.h"
#include "util.h"
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/mman.h> /* mmap() is defined in this header */


#define DEBUG 0
#define VERIFY 0
#define CHECK_DIS 0
#define CODEGEN_V2 1
#define THREAD_POOL_SIZE 0

#define LOADING_LIMIT 100000
#define PROCESSING_LIMIT 100000
using namespace google::protobuf::io;
using namespace boost::filesystem;


bool sendZ3Solver(bool opti, Z3Solver &g_solver, std::shared_ptr<JitCmdv2> cmd,
    std::unordered_map<uint32_t,uint8_t> &solu, uint64_t *st);

bool readDelimitedFrom(
    google::protobuf::io::ZeroCopyInputStream* rawInput,
    google::protobuf::MessageLite* message) {
  // We create a new coded stream for each message.  Don't worry, this is fast,
  // and it makes sure the 64MB total size limit is imposed per-message rather
  // than on the whole stream.  (See the CodedInputStream interface for more
  // info on this limit.)
  google::protobuf::io::CodedInputStream input(rawInput);
  input.SetRecursionLimit(10000);

  // Read the size.
  uint32_t size;
  if (!input.ReadVarint32(&size)) return false;

  // Tell the stream not to read beyond that size.
  google::protobuf::io::CodedInputStream::Limit limit =
    input.PushLimit(size);

  // Parse the message.
  if (!message->MergeFromCodedStream(&input)) return false;
  if (!input.ConsumedEntireMessage()) return false;

  // Release the limit.
  input.PopLimit(limit);

  return true;
}


Z3Solver g_solver;


bool z3Task(std::shared_ptr<JitCmdv2> cmd) {
  bool suc = false;
  uint64_t st = 0;
  std::unordered_map<uint32_t,uint8_t> solu;		
  suc = sendZ3Solver(false,g_solver,cmd, solu, &st);
  return suc;
}


int main(int argc, char** argv) {
  int count = 0;
  int suc_count = 0;
  for (directory_entry& entry : directory_iterator(argv[1])) {
    int fd = open(entry.path().c_str(),O_RDONLY);
    ZeroCopyInputStream* rawInput = new google::protobuf::io::FileInputStream(fd);
    bool suc = false;
    do {
      std::shared_ptr<JitCmdv2> cmd =  std::make_shared<JitCmdv2>();
      suc = readDelimitedFrom(rawInput,cmd.get());
      if (suc) {
        if (z3Task(cmd)) 
          suc_count++;
        count++;
      } else {
        break;
      }
    } while(suc);
    delete rawInput;
    close(fd);
  }
  printf("handled %d constraints, solved %d\n", count, suc_count);
  return 0;
}
