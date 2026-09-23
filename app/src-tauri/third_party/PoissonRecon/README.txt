Vendored from https://github.com/mkazhdan/PoissonRecon
commit 262b0f539d404057d1f36e1adc07fc9388678899 (2026-04-29), version 18.76.
Only the header-only reconstruction core (Src/*.h, Src/*.inl) is kept; the
executables, image codecs and zlib are not needed. MIT licensed, see LICENSE.

Local change: ThreadPool::SetNumThreads() added in MultiThreading.h.
