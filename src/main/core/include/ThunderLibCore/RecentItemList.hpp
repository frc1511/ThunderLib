#pragma once

#include <wpi/static_circular_buffer.h>
#include <cstddef>
#include <algorithm>

namespace thunder::core {

template <typename T, size_t N>
struct RecentItemList {
  using ListType = wpi::static_circular_buffer<T, N>;
  ListType m_files;

 public:
  void add(const T& item) {
    remove(item);
    m_files.push_front(item);
  }

  bool remove(const T& item) {
    size_t itemIndex = 0;
    bool found = false;
    for (; itemIndex < m_files.size(); itemIndex++) {
      if (m_files[itemIndex] == item) {
        found = true;
        break;
      }
    }
    if (!found)
      return false;

    if (itemIndex > 0) {
      typename ListType::iterator shiftBegin(&m_files, 0);
      typename ListType::iterator shiftEnd(&m_files, itemIndex + 1);
      std::shift_right(shiftBegin, shiftEnd, 1);
    }

    m_files.pop_front();
    return true;
  }

  void clear() { m_files.reset(); }

  using value_type = T;

  using const_iterator = ListType::const_iterator;
  using const_reverse_iterator = ListType::const_reverse_iterator;

  const_iterator begin() const { return m_files.begin(); }
  const_iterator end() const { return m_files.end(); }
  const_iterator cbegin() const { return m_files.begin(); }
  const_iterator cend() const { return m_files.end(); }
  const_reverse_iterator rbegin() const { return m_files.rbegin(); }
  const_reverse_iterator rend() const { return m_files.rend(); }
  const_reverse_iterator crbegin() const { return m_files.crbegin(); }
  const_reverse_iterator crend() const { return m_files.crend(); }

  size_t size() const { return m_files.size(); }

  bool empty() const { return size() == 0; }

  const T& front() { return m_files.front(); }
  const T& back() { return m_files.back(); }

  const T& operator[](size_t index) const { return m_files[index]; }
};

}  // namespace thunder::core
