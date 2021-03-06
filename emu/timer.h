#pragma once

struct WorkItem {
  WorkItem(callback_t callback) : _callback(callback) {}

  void operator()(void) { _callback(); }

 private:
  callback_t _callback;
};

struct TimerItem : public WorkItem {
  TimerItem(Time timeout, callback_t callback, bool periodic = false)
      : WorkItem(callback),
        _timeout(timeout),
        _periodic(periodic),
        _deadline(time_zero) {}

  bool expired(Time time) { return _deadline <= time; }

  const Time deadline(void) { return _deadline; }

  void schedule(Time abs) { _deadline = abs + _timeout; }

  bool periodic(void) { return _periodic; }

 private:
  Time _timeout;
  bool _periodic;
  Time _deadline;
};

typedef std::shared_ptr<TimerItem> TimerItem_ptr;

class TimerQueue {
 public:
  TimerQueue(void);
  ~TimerQueue(void);

  void stop(void);
  Time run(Time delta);

  TimerItem_ptr add_periodic(Time period, callback_t callback);
  TimerItem_ptr add_timeout(Time timeout, callback_t callback);
  bool remove(TimerItem_ptr timer);

 private:
  void add(TimerItem_ptr timer);

  TimerItem_ptr pop(Time deadline);
  void wait(void);

  Time _clock;

  std::mutex mtx;
  std::condition_variable cv;

  std::list<TimerItem_ptr> _timers;

  bool _quit;
};
