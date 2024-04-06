// Copyright 2024 Homalozoa. All rights reserved.
// Copyright 2020 Michael Spieler. All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the Farbod nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.


#include "gtest/gtest.h"
#include "ocs2_core/thread_support/ThreadPool.hpp"

TEST(testThreadPool, testCanExecuteTask)
{
  ocs2::ThreadPool pool(1);
  std::future<void> res;
  int answer = 0;

  res = pool.run([&answer](int) { answer = 42; });
  res.wait_for(std::chrono::seconds(1));

  EXPECT_EQ(answer, 42);
}

TEST(testThreadPool, testReturnType)
{
  ocs2::ThreadPool pool(1);
  std::future<int> res;

  res = pool.run([](int) -> int { return 42; });

  EXPECT_EQ(res.get(), 42);
}

TEST(testThreadPool, testPropagateException)
{
  ocs2::ThreadPool pool(1);
  std::function<void(int)> task;

  // send task to pool
  task = [](int) { throw std::string("exception"); };
  EXPECT_THROW(pool.run(task).get(), std::string);
}

TEST(testThreadPool, testCanExecuteMultipleTasks)
{
  ocs2::ThreadPool pool(2);
  std::function<void(int)> task;
  std::future<void> res1, res2;

  std::promise<void> barrier_promise;
  std::shared_future<void> barrier = barrier_promise.get_future();

  std::string data1, data2;

  res1 = pool.run([&data1, barrier](int) {
    data1 = "running";
    barrier.wait();
    data1 = "done";
  });
  res2 = pool.run([&data2, barrier](int) {
    data2 = "running";
    barrier.wait();
    data2 = "done";
  });

  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  // check that both threads are blocking
  EXPECT_EQ(data1, "running");
  EXPECT_EQ(data2, "running");

  // signal threads
  barrier_promise.set_value();

  // wait for pool to run tasks
  std::future_status status1 = res1.wait_for(std::chrono::seconds(1));
  ASSERT_EQ(status1, std::future_status::ready);

  std::future_status status2 = res2.wait_for(std::chrono::seconds(1));
  ASSERT_EQ(status2, std::future_status::ready);

  EXPECT_EQ(data1, "done");
  EXPECT_EQ(data2, "done");

  res1.get();
  res2.get();
}

TEST(testThreadPool, testRunMultiple)
{
  ocs2::ThreadPool pool(2);
  std::atomic_int counter;
  counter = 0;

  pool.runParallel([&](int) { counter++; }, 42);

  EXPECT_EQ(counter, 42);
}

TEST(testThreadPool, testNoThreads)
{
  ocs2::ThreadPool pool(0);

  auto fut1 = pool.run([&](int) -> std::string { return "runs on main thread"; });
  EXPECT_EQ(fut1.get(), "runs on main thread");
}

TEST(testThreadPool, testRunMultipleNoThreads)
{
  ocs2::ThreadPool pool(0);
  std::atomic_int counter;
  counter = 0;

  pool.runParallel([&](int) { counter++; }, 42);

  EXPECT_EQ(counter, 42);
}

TEST(testThreadPool, testMoveOnlyTask)
{
  ocs2::ThreadPool pool(2);

  struct MoveOnlyTask
  {
    MoveOnlyTask() = default;
    ~MoveOnlyTask() = default;
    MoveOnlyTask(const MoveOnlyTask &) = delete;
    MoveOnlyTask & operator=(const MoveOnlyTask &) = delete;
    MoveOnlyTask(MoveOnlyTask &&) = default;
    MoveOnlyTask & operator=(MoveOnlyTask &&) = default;
    double operator()(int) { return 3.14; }
  };

  auto f = MoveOnlyTask();

  auto result = pool.run(std::move(f));

  EXPECT_EQ(result.get(), 3.14);
}
