#ifndef Z3_SOLVER_H_
#define Z3_SOLVER_H_

#include "rgd.pb.h"
using namespace rgd;
#include <z3++.h>
#include <fstream>
#include <sys/time.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <vector>


extern z3::context g_z3_context;

class Z3Solver {
public:
  Z3Solver();
  void add(z3::expr expr);
	void reset();
  bool check(std::unordered_map<uint32_t,uint8_t> &solu);
  bool checkonly();
	z3::expr serialize(const JitRequest* req, 
						std::unordered_map<uint32_t,z3::expr> &expr_cache);
	z3::expr serialize_with_solution(const JitRequest* req, 
						std::unordered_map<uint32_t,z3::expr> &expr_cache,
						std::unordered_map<uint32_t,uint8_t> &solu);

  //bool checkAndSave(const std::string& postfix="");
  uint8_t getInput(uint32_t index);


protected:
  z3::context          context_;
  z3::solver            solver_;
  uint64_t              start_time_;
  uint64_t              solving_time_;
  uint64_t              solving_count_;

  std::vector<uint8_t> getConcreteValues();

	inline z3::expr cache_expr(uint32_t label, z3::expr const &e, 
						std::unordered_map<uint32_t,z3::expr> &expr_cache);

};

//extern Z3Solver* g_solver;


#endif // Z3_SOLVER_H_
