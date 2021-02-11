#include <set>
#include <byteswap.h>
#include "z3solver.h"

#include "rgd_op.h"
#include "rgd.pb.h"
#include "util.h"

#include <sys/types.h>
#include <sys/stat.h>
#include <sys/mman.h> /* mmap() is defined in this header */
#include <fcntl.h>
#include <unistd.h>
using namespace rgd;
void printExpression(const JitRequest* req);
uint64_t getTimeStamp();
const int kSessionIdLength = 32;
//const unsigned kSolverTimeout = 5000; // 10 seconds
const unsigned kSolverTimeout = 10000; // 10 seconds

Z3Solver::Z3Solver() :
	solver_(z3::solver(context_, "QF_BV"))
	, start_time_(getTimeStamp())
	, solving_time_(0)
	, solving_count_(0)
{
	// Set timeout for solver
	z3::params p(context_);
	p.set(":timeout", kSolverTimeout);
	solver_.set(p);
}

void Z3Solver::add(z3::expr expr) {
	if (!expr.is_const())
		solver_.add(expr.simplify());
}

void Z3Solver::reset() {
	solver_.reset();
}

bool Z3Solver::checkonly() {
	static int count = 0;
	z3::check_result res;
	try {
		res = solver_.check();
		if (res != z3::sat) {
			//std::cerr << "branch NOT solved in checkonly!" << std::endl;
			//assert(false && "not pass check!");
			//std::cout << "f count " << ++count << std::endl;
			return false;
		}
	} catch (z3::exception e) {
		std::cerr << "Z3 alert: " << e.msg() << std::endl;
		return false;
	}
	return true;
}


bool Z3Solver::check(std::unordered_map<uint32_t,uint8_t> &solu) {
	z3::check_result res;
	try {
		res = solver_.check();
		if (res==z3::sat) {
			z3::model m = solver_.get_model();

			unsigned num_constants = m.num_consts();
			for(unsigned i = 0; i< num_constants; i++) {
				z3::func_decl decl = m.get_const_decl(i);
				z3::expr e = m.get_const_interp(decl);
				z3::symbol name = decl.name();
				if(name.kind() == Z3_INT_SYMBOL) {
					uint8_t value = (uint8_t)e.get_numeral_int();
					solu[name.to_int()] = value;
				//	std::cout << " generate_input index is " << name.to_int() << " and value is " << (int)value << std::endl;
				}
			}

			return true;
		}	else {
			//std::cerr << "branch NOT solved in check()" << std::endl;
			return false;
		}
	} catch (z3::exception e) {
		std::cerr << "Z3 alert: " << e.msg() << std::endl;
	}
}

inline z3::expr Z3Solver::cache_expr(uint32_t label, z3::expr const &e, 
		std::unordered_map<uint32_t,z3::expr> &expr_cache) {	
	if (label!=0)
		expr_cache.insert({label,e});
	return e;
}

z3::expr Z3Solver::serialize_with_solution(const JitRequest* req,
		std::unordered_map<uint32_t,z3::expr> &expr_cache,
		std::unordered_map<uint32_t,uint8_t> &solu) {

	auto itr = expr_cache.find(req->label());

	if (req->label() != 0 && itr != expr_cache.end())
		return itr->second;
	switch (req->kind()) {
		case rgd::Bool: {
			// getTrue is actually 1 bit integer 1
			return cache_expr(req->label(),context_.bool_val(req->boolvalue()),expr_cache);
		}
		case rgd::Constant: {
			if (req->bits() == 1) {
				return cache_expr(req->label(),context_.bool_val(req->value()=="1"),expr_cache);
			}
			else
				return cache_expr(req->label(),context_.bv_val(req->value().c_str(),req->bits()),expr_cache);
		}
		case rgd::Read: {
			//z3::symbol symbol = context_.int_symbol(req->index());
			uint64_t v = 0;
			for(int i=0;i<req->bits()/8;i++) {
					v = v | (((uint64_t)solu[req->index() + i]) << (i*8));
			}
			//uint8_t v = solu[req->index()];
			return cache_expr(req->label(),context_.bv_val(v,req->bits()),expr_cache);
			//return cache_expr(req->label(),context_.constant(symbol,sort),expr_cache);
		}
		case rgd::Concat: {
			z3::expr c1 = serialize_with_solution(&req->children(0),expr_cache,solu);
			z3::expr c2 = serialize_with_solution(&req->children(1),expr_cache,solu);
			return cache_expr(req->label(),z3::concat(c2,c1),expr_cache);
		}
		case rgd::Extract: {
			z3::expr c1 = serialize_with_solution(&req->children(0),expr_cache,solu);
			return cache_expr(req->label(),c1.extract(req->index()+req->bits()-1,req->index()),expr_cache);
		}
		case rgd::ZExt: {
			z3::expr c1 = serialize_with_solution(&req->children(0),expr_cache,solu);
			z3::zext(c1,req->bits()-req->children(0).bits());
			return cache_expr(req->label(),z3::zext(c1,req->bits()-req->children(0).bits()),expr_cache);
		}
		case rgd::SExt: {
			z3::expr c1 = serialize_with_solution(&req->children(0),expr_cache,solu);
			return cache_expr(req->label(),z3::sext(c1,req->bits()-req->children(0).bits()),expr_cache);
		}
		case rgd::Add: {
			z3::expr c1 = serialize_with_solution(&req->children(0),expr_cache,solu);
			z3::expr c2 = serialize_with_solution(&req->children(1),expr_cache,solu);
			return cache_expr(req->label(),c1+c2,expr_cache);
		}
		case rgd::Sub: {
			z3::expr c1 = serialize_with_solution(&req->children(0),expr_cache,solu);
			z3::expr c2 = serialize_with_solution(&req->children(1),expr_cache,solu);
			return cache_expr(req->label(),c1-c2,expr_cache);
		}
		case rgd::Mul: {
			z3::expr c1 = serialize_with_solution(&req->children(0),expr_cache,solu);
			z3::expr c2 = serialize_with_solution(&req->children(1),expr_cache,solu);
			return cache_expr(req->label(),c1*c2,expr_cache);
		}
		case rgd::UDiv: {
			z3::expr c1 = serialize_with_solution(&req->children(0),expr_cache,solu);
			z3::expr c2 = serialize_with_solution(&req->children(1),expr_cache,solu);
			return cache_expr(req->label(),z3::udiv(c1,c2),expr_cache);
		}
		case rgd::SDiv: {
			z3::expr c1 = serialize_with_solution(&req->children(0),expr_cache,solu);
			z3::expr c2 = serialize_with_solution(&req->children(1),expr_cache,solu);
			return cache_expr(req->label(),c1/c2,expr_cache); 
		} 
		case rgd::URem: {
			z3::expr c1 = serialize_with_solution(&req->children(0),expr_cache,solu);
			z3::expr c2 = serialize_with_solution(&req->children(1),expr_cache,solu);
			return cache_expr(req->label(),z3::urem(c1,c2),expr_cache);
		}
		case rgd::SRem: {
			z3::expr c1 = serialize_with_solution(&req->children(0),expr_cache,solu);
			z3::expr c2 = serialize_with_solution(&req->children(1),expr_cache,solu);
			return cache_expr(req->label(),z3::srem(c1,c2),expr_cache);
		}
		case rgd::Neg: {
			z3::expr c1 = serialize_with_solution(&req->children(0),expr_cache,solu);
			return cache_expr(req->label(),-c1,expr_cache);
		}
		case rgd::Not: {
			z3::expr c1 = serialize_with_solution(&req->children(0),expr_cache,solu);
			return cache_expr(req->label(),~c1,expr_cache);
		}
		case rgd::And: {
			z3::expr c1 = serialize_with_solution(&req->children(0),expr_cache,solu);
			z3::expr c2 = serialize_with_solution(&req->children(1),expr_cache,solu);
			return cache_expr(req->label(),c1 & c2,expr_cache);
		}
		case rgd::Or: {
			z3::expr c1 = serialize_with_solution(&req->children(0),expr_cache,solu);
			z3::expr c2 = serialize_with_solution(&req->children(1),expr_cache,solu);
			return cache_expr(req->label(),c1 | c2,expr_cache);
		}
		case rgd::Xor: {
			z3::expr c1 = serialize_with_solution(&req->children(0),expr_cache,solu);
			z3::expr c2 = serialize_with_solution(&req->children(1),expr_cache,solu);
			return cache_expr(req->label(),c1^c2,expr_cache);
		}
		case rgd::Shl: {
			z3::expr c1 = serialize_with_solution(&req->children(0),expr_cache,solu);
			z3::expr c2 = serialize_with_solution(&req->children(1),expr_cache,solu);
			return cache_expr(req->label(),z3::shl(c1,c2),expr_cache);
		}
		case rgd::LShr: {
			z3::expr c1 = serialize_with_solution(&req->children(0),expr_cache,solu);
			z3::expr c2 = serialize_with_solution(&req->children(1),expr_cache,solu);
			return cache_expr(req->label(),z3::lshr(c1,c2),expr_cache);
		}
		case rgd::AShr: {
			z3::expr c1 = serialize_with_solution(&req->children(0),expr_cache,solu);
			z3::expr c2 = serialize_with_solution(&req->children(1),expr_cache,solu);
			return cache_expr(req->label(),z3::ashr(c1,c2),expr_cache);
		}
		case rgd::Equal: {
			z3::expr c1 = serialize_with_solution(&req->children(0),expr_cache,solu);
			z3::expr c2 = serialize_with_solution(&req->children(1),expr_cache,solu);
			return cache_expr(req->label(),c1==c2,expr_cache);
		}
		case rgd::Distinct: {
			z3::expr c1 = serialize_with_solution(&req->children(0),expr_cache,solu);
			z3::expr c2 = serialize_with_solution(&req->children(1),expr_cache,solu);
			z3::expr rr = c1!=c2;
			return cache_expr(req->label(),c1!=c2,expr_cache);
		}
		case rgd::Ult: {
			z3::expr c1 = serialize_with_solution(&req->children(0),expr_cache,solu);
			z3::expr c2 = serialize_with_solution(&req->children(1),expr_cache,solu);
			return cache_expr(req->label(),z3::ult(c1,c2),expr_cache);
		}
		case rgd::Ule: {
			z3::expr c1 = serialize_with_solution(&req->children(0),expr_cache,solu);
			z3::expr c2 = serialize_with_solution(&req->children(1),expr_cache,solu);
			return cache_expr(req->label(),z3::ule(c1,c2),expr_cache);
		}
		case rgd::Ugt: {
			z3::expr c1 = serialize_with_solution(&req->children(0),expr_cache,solu);
			z3::expr c2 = serialize_with_solution(&req->children(1),expr_cache,solu);
			return cache_expr(req->label(),z3::ugt(c1,c2),expr_cache);
		}
		case rgd::Uge: {
			z3::expr c1 = serialize_with_solution(&req->children(0),expr_cache,solu);
			z3::expr c2 = serialize_with_solution(&req->children(1),expr_cache,solu);
			return cache_expr(req->label(),z3::uge(c1,c2),expr_cache);
		}
		case rgd::Slt: {
			z3::expr c1 = serialize_with_solution(&req->children(0),expr_cache,solu);
			z3::expr c2 = serialize_with_solution(&req->children(1),expr_cache,solu);
			return cache_expr(req->label(),c1<c2,expr_cache);
		}
		case rgd::Sle: {
			z3::expr c1 = serialize_with_solution(&req->children(0),expr_cache,solu);
			z3::expr c2 = serialize_with_solution(&req->children(1),expr_cache,solu);
			return cache_expr(req->label(),c1<=c2,expr_cache);
		}
		case rgd::Sgt: {
			z3::expr c1 = serialize_with_solution(&req->children(0),expr_cache,solu);
			z3::expr c2 = serialize_with_solution(&req->children(1),expr_cache,solu);
			return cache_expr(req->label(),c1>c2,expr_cache);
		}
		case rgd::Sge: {
			z3::expr c1 = serialize_with_solution(&req->children(0),expr_cache,solu);
			z3::expr c2 = serialize_with_solution(&req->children(1),expr_cache,solu);
			return cache_expr(req->label(),c1>=c2,expr_cache);
		}
		case rgd::LOr: {
			z3::expr c1 = serialize_with_solution(&req->children(0),expr_cache,solu);
			z3::expr c2 = serialize_with_solution(&req->children(1),expr_cache,solu);
			return cache_expr(req->label(),c1 || c2,expr_cache);
		}
		case rgd::LAnd: {
			z3::expr c1 = serialize_with_solution(&req->children(0),expr_cache,solu);
			z3::expr c2 = serialize_with_solution(&req->children(1),expr_cache,solu);
			return cache_expr(req->label(),c1 && c2,expr_cache);
		}
		case rgd::LNot: {
			z3::expr c1 = serialize_with_solution(&req->children(0),expr_cache,solu);
			return cache_expr(req->label(),!c1,expr_cache);
		}
		default:
			std::cerr << "WARNING: unhandler expr: ";
			break;
	}
}

z3::expr Z3Solver::serialize(const JitRequest* req,
		std::unordered_map<uint32_t,z3::expr> &expr_cache) {

	auto itr = expr_cache.find(req->label());

	if (req->label() != 0 && itr != expr_cache.end())
		return itr->second;
	switch (req->kind()) {
		case rgd::Bool: {
			// getTrue is actually 1 bit integer 1
			return cache_expr(req->label(),context_.bool_val(req->boolvalue()),expr_cache);
		}
		case rgd::Constant: {
			if (req->bits() == 1) {
				return cache_expr(req->label(),context_.bool_val(req->value()=="1"),expr_cache);
			}
			return cache_expr(req->label(),context_.bv_val(req->value().c_str(),req->bits()),expr_cache);
		}
		case rgd::Read: {
			z3::symbol symbol = context_.int_symbol(req->index());
			z3::sort sort = context_.bv_sort(8);
			z3::expr out = context_.constant(symbol,sort);
			for(uint32_t i=1; i<req->bits()/8; i++) {
				symbol = context_.int_symbol(req->index()+i);
				out = z3::concat(context_.constant(symbol,sort),out);
			}
			return cache_expr(req->label(),out,expr_cache);
		}
		case rgd::Concat: {
			z3::expr c1 = serialize(&req->children(0),expr_cache);
			z3::expr c2 = serialize(&req->children(1),expr_cache);
			return cache_expr(req->label(),z3::concat(c2,c1),expr_cache);
		}
		case rgd::Extract: {
			z3::expr c1 = serialize(&req->children(0),expr_cache);
			return cache_expr(req->label(),c1.extract(req->index()+req->bits()-1,req->index()),expr_cache);
		}
		case rgd::ZExt: {
			z3::expr c1 = serialize(&req->children(0),expr_cache);
			if (c1.is_bool())
				c1 = z3::ite(c1,context_.bv_val(1,1),context_.bv_val(0,1));
			uint32_t cbits = c1.get_sort().bv_size();
			return cache_expr(req->label(),z3::zext(c1,req->bits()-cbits),expr_cache);
		}
		case rgd::SExt: {
			z3::expr c1 = serialize(&req->children(0),expr_cache);
			return cache_expr(req->label(),z3::sext(c1,req->bits()-req->children(0).bits()),expr_cache);
		}
		case rgd::Add: {
			z3::expr c1 = serialize(&req->children(0),expr_cache);
			z3::expr c2 = serialize(&req->children(1),expr_cache);
			return cache_expr(req->label(),c1+c2,expr_cache);
		}
		case rgd::Sub: {
			z3::expr c1 = serialize(&req->children(0),expr_cache);
			z3::expr c2 = serialize(&req->children(1),expr_cache);
			return cache_expr(req->label(),c1-c2,expr_cache);
		}
		case rgd::Mul: {
			z3::expr c1 = serialize(&req->children(0),expr_cache);
			z3::expr c2 = serialize(&req->children(1),expr_cache);
			return cache_expr(req->label(),c1*c2,expr_cache);
		}
		case rgd::UDiv: {
			z3::expr c1 = serialize(&req->children(0),expr_cache);
			z3::expr c2 = serialize(&req->children(1),expr_cache);
			return cache_expr(req->label(),z3::udiv(c1,c2),expr_cache);
		}
		case rgd::SDiv: {
			z3::expr c1 = serialize(&req->children(0),expr_cache);
			z3::expr c2 = serialize(&req->children(1),expr_cache);
			return cache_expr(req->label(),c1/c2,expr_cache); 
		} 
		case rgd::URem: {
			z3::expr c1 = serialize(&req->children(0),expr_cache);
			z3::expr c2 = serialize(&req->children(1),expr_cache);
			return cache_expr(req->label(),z3::urem(c1,c2),expr_cache);
		}
		case rgd::SRem: {
			z3::expr c1 = serialize(&req->children(0),expr_cache);
			z3::expr c2 = serialize(&req->children(1),expr_cache);
			return cache_expr(req->label(),z3::srem(c1,c2),expr_cache);
		}
		case rgd::Neg: {
			z3::expr c1 = serialize(&req->children(0),expr_cache);
			return cache_expr(req->label(),-c1,expr_cache);
		}
		case rgd::Not: {
			z3::expr c1 = serialize(&req->children(0),expr_cache);
			return cache_expr(req->label(),~c1,expr_cache);
		}
		case rgd::And: {
			z3::expr c1 = serialize(&req->children(0),expr_cache);
			z3::expr c2 = serialize(&req->children(1),expr_cache);
			return cache_expr(req->label(),c1 & c2,expr_cache);
		}
		case rgd::Or: {
			z3::expr c1 = serialize(&req->children(0),expr_cache);
			z3::expr c2 = serialize(&req->children(1),expr_cache);
			return cache_expr(req->label(),c1 | c2,expr_cache);
		}
		case rgd::Xor: {
			z3::expr c1 = serialize(&req->children(0),expr_cache);
			z3::expr c2 = serialize(&req->children(1),expr_cache);
			return cache_expr(req->label(),c1^c2,expr_cache);
		}
		case rgd::Shl: {
			z3::expr c1 = serialize(&req->children(0),expr_cache);
			z3::expr c2 = serialize(&req->children(1),expr_cache);
			return cache_expr(req->label(),z3::shl(c1,c2),expr_cache);
		}
		case rgd::LShr: {
			z3::expr c1 = serialize(&req->children(0),expr_cache);
			z3::expr c2 = serialize(&req->children(1),expr_cache);
			return cache_expr(req->label(),z3::lshr(c1,c2),expr_cache);
		}
		case rgd::AShr: {
			z3::expr c1 = serialize(&req->children(0),expr_cache);
			z3::expr c2 = serialize(&req->children(1),expr_cache);
			return cache_expr(req->label(),z3::ashr(c1,c2),expr_cache);
		}
		case rgd::Equal: {
			z3::expr c1 = serialize(&req->children(0),expr_cache);
			z3::expr c2 = serialize(&req->children(1),expr_cache);
			return cache_expr(req->label(),c1==c2,expr_cache);
		}
		case rgd::Distinct: {
			z3::expr c1 = serialize(&req->children(0),expr_cache);
			z3::expr c2 = serialize(&req->children(1),expr_cache);
			z3::expr rr = c1!=c2;
			return cache_expr(req->label(),c1!=c2,expr_cache);
		}
		case rgd::Ult: {
			z3::expr c1 = serialize(&req->children(0),expr_cache);
			z3::expr c2 = serialize(&req->children(1),expr_cache);
			return cache_expr(req->label(),z3::ult(c1,c2),expr_cache);
		}
		case rgd::Ule: {
			z3::expr c1 = serialize(&req->children(0),expr_cache);
			z3::expr c2 = serialize(&req->children(1),expr_cache);
			return cache_expr(req->label(),z3::ule(c1,c2),expr_cache);
		}
		case rgd::Ugt: {
			z3::expr c1 = serialize(&req->children(0),expr_cache);
			z3::expr c2 = serialize(&req->children(1),expr_cache);
			return cache_expr(req->label(),z3::ugt(c1,c2),expr_cache);
		}
		case rgd::Uge: {
			z3::expr c1 = serialize(&req->children(0),expr_cache);
			z3::expr c2 = serialize(&req->children(1),expr_cache);
			return cache_expr(req->label(),z3::uge(c1,c2),expr_cache);
		}
		case rgd::Slt: {
			z3::expr c1 = serialize(&req->children(0),expr_cache);
			z3::expr c2 = serialize(&req->children(1),expr_cache);
			return cache_expr(req->label(),c1<c2,expr_cache);
		}
		case rgd::Sle: {
			z3::expr c1 = serialize(&req->children(0),expr_cache);
			z3::expr c2 = serialize(&req->children(1),expr_cache);
			return cache_expr(req->label(),c1<=c2,expr_cache);
		}
		case rgd::Sgt: {
			z3::expr c1 = serialize(&req->children(0),expr_cache);
			z3::expr c2 = serialize(&req->children(1),expr_cache);
			return cache_expr(req->label(),c1>c2,expr_cache);
		}
		case rgd::Sge: {
			z3::expr c1 = serialize(&req->children(0),expr_cache);
			z3::expr c2 = serialize(&req->children(1),expr_cache);
			return cache_expr(req->label(),c1>=c2,expr_cache);
		}
		case rgd::LOr: {
			z3::expr c1 = serialize(&req->children(0),expr_cache);
			z3::expr c2 = serialize(&req->children(1),expr_cache);
			return cache_expr(req->label(),c1 || c2,expr_cache);
		}
		case rgd::LAnd: {
			z3::expr c1 = serialize(&req->children(0),expr_cache);
			z3::expr c2 = serialize(&req->children(1),expr_cache);
			return cache_expr(req->label(),c1 && c2,expr_cache);
		}
		case rgd::LNot: {
			z3::expr c1 = serialize(&req->children(0),expr_cache);
			return cache_expr(req->label(),!c1,expr_cache);
		}
		default:
			std::cerr << "WARNING: unhandler expr: ";
			break;
	}
}

bool checkSat(Z3Solver &g_solver, std::shared_ptr<JitCmdv2> cmd) {
	//deprecating reset and solve
	assert(cmd->cmd()==2);
	g_solver.reset();
	for (int i = 0; i < cmd->expr_size(); i++) {
		std::unordered_map<uint32_t,z3::expr> expr_cache;
		const JitRequest *req = &cmd->expr(i);
		try {
			z3::expr z3expr = g_solver.serialize(req,expr_cache);
			g_solver.add(z3expr);
		} catch (z3::exception e) {
			std::cout << "z3 alert: " << e.msg() << std::endl;
			return false;
		}
	}
	return g_solver.checkonly();
}


bool sendZ3Solver(bool opti, Z3Solver &g_solver, std::shared_ptr<JitCmdv2> cmd,
									std::unordered_map<uint32_t,uint8_t> &solu, uint64_t *st) {
	//deprecating reset and solve
	assert(cmd->cmd()==2);
	g_solver.reset();
	//printExpression(&cmd->expr(0));
	static int count = 0;
	int num_expr = 0;
	if (opti)
		num_expr = 1;
	else
		num_expr = cmd->expr_size();
	for (int i = 0; i < num_expr; i++) {
//	for (int i = 0; i < 1; i++) {
		std::unordered_map<uint32_t,z3::expr> expr_cache;
		const JitRequest *req = &cmd->expr(i);
		try {
			z3::expr z3expr = g_solver.serialize(req,expr_cache);
			g_solver.add(z3expr);
		} catch (z3::exception e) {
			std::cout << "z3 alert: " << e.msg() << std::endl;
			return false;
		}
	}
	uint64_t start = getTimeStamp();
	bool suc =  g_solver.check(solu);
	uint64_t solving =  getTimeStamp() - start;
	*st = solving;
#if 0
	if (!suc)  {
		g_solver.reset();
		const JitRequest *req = &cmd->expr(0);
		try {
		std::unordered_map<uint32_t,z3::expr> expr_cache;
		z3::expr z3expr = g_solver.serialize(req,expr_cache);
		g_solver.add(z3expr);
		} catch (z3::exception e) {
			std::cout << "z3 alert: " << e.msg() << std::endl;
			return false;
		}
		suc =  g_solver.check(solu);
	}
#endif
	return suc;
}

void sendZ3Solver_with_solution(Z3Solver &g_solver, std::shared_ptr<JitCmdv2> cmd,
		std::unordered_map<uint32_t,uint8_t> &solu) {
	assert(cmd->cmd()==2);
	if (solu.size() == 0) return;
	//if (!checkSat(g_solver,cmd)) return;
	static int count  =0;
	g_solver.reset();

//	for(auto itr = solu.begin();itr!=solu.end();itr++)
//		std::cout << "try z3 with index " << itr->first << " and value " << (int)itr->second << std::endl;
	for (int i = 0; i < cmd->expr_size(); i++) {
		std::unordered_map<uint32_t,z3::expr> expr_cache;
		const JitRequest *req = &cmd->expr(i);
		//printExpression(req);
		try {
			z3::expr z3expr = g_solver.serialize_with_solution(req,expr_cache,solu);
			g_solver.add(z3expr);
		} catch (z3::exception e) {
			//std::cout << "z3 alert: " << e.msg() << std::endl;
		}
	}
	if (!g_solver.checkonly()) {
		 std::cout << "failed z3 test" << ++count << std::endl;
	//		saveRequest(*cmd,"failed.data");
//			printf("num subexprs is %d, file name is %s\n",cmd->expr_size(),cmd->file_name().c_str());
			for(int i =0;i<1;i++)
				printExpression(&cmd->expr(i));
	//		assert(false && "not pass check!");
	//	printExpression(&cmd->expr(0));
	//		saveRequest(*cmd, "failed.data");
	}
#if 0
	//std::cout << "try the same thing with z3" << std::endl;

	g_solver.reset();
	std::unordered_map<uint32_t,uint8_t> solu1;
	for (int i = 0; i < 1; i++) {
		std::unordered_map<uint32_t,z3::expr> expr_cache;
		const JitRequest *req = &cmd->expr(i);
		//printExpression(req);
		try {
			z3::expr z3expr = g_solver.serialize(req,expr_cache);
			g_solver.add(z3expr);
		} catch (z3::exception e) {
			std::cout << "z3 alert: " << e.msg() << std::endl;
		}
	}
	g_solver.check(solu1);

		for (auto it=solu1.begin();it!=solu1.end();it++) {
			printf("Z3 generate_input index is %u and value is %x\n", it->first,(uint32_t)it->second);
		}
#endif 
}



