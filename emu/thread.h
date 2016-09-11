#pragma once

class LWPFiber {
 public:
  LWPFiber(void);
  ~LWPFiber(void);

  bool runnable(void) { return m_runnable; }

 private:
  bool m_runnable;
  exec_buf_t m_stack;
};

typedef std::unique_ptr<LWPFiber> LWPFiber_ptr;

class LWPScheduler : public std::thread {
 public:
  LWPScheduler(void);
  virtual ~LWPScheduler(void);

  void run(void);

 private:
  std::list<LWPFiber_ptr> m_threads;
};
