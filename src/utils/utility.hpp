#ifndef __DG_UTILS_UTILITY__
#define __DG_UTILS_UTILITY__

#include <vector>
#include <string>
#include <assert.h>

namespace dg
{

    /**
     * @brief FIFO(First In First Out) 링버퍼 구현 by jylee
     *
     * 가장 최근의 n개의 데이터만을 유지하기 위한 자료구조 (템플릿 구현)
     */
    template <class T>
    class RingBuffer
    {
    public:
        /**
        * 기본 생성자 (empty 링버퍼 생성)
        */
        RingBuffer();

        /**
        * 생성자
        * @param size 링버퍼 최대 크기 (유지할 데이터의 개수)
        */
        RingBuffer(int size);

        /** 소멸자 */
        virtual ~RingBuffer();

        /**
        * 링버퍼의 데이터 존재 여부를 반환
        * @return Return True if the buffer is empty
        */
        bool empty() const;

        /**
        * 데이터를 링버퍼 끝에 삽입 (버퍼 최대크기에 도달할 경우 가장 오래된 데이터를 제거)
        * @param data 추가할 데이터
        */
        virtual bool push_back(const T& data);

        /**
        * 데이터를 링버퍼 중간에 삽입 (버퍼 최대크기에 도달할 경우 가장 오래된 데이터를 제거)
        * @param data 삽입할 데이터
        * @return Index of newly inserted data if succeeds (return -1 if failed)
        */
        virtual int insert(int index, const T& data);

        /**
        * 데이터 구간을 삭제
        * @param first_idx Index of first element to erase
        * @param last_idx Index of last element to erase (if it is given -1, erase to the end)
        * @return Return True if successful
        */
        virtual bool erase(int first_idx, int last_idx = -1);

        /**
        * 링버퍼의 데이터 개수(현재 저장된 데이터 개수)를 반환
        * @return 링버퍼 데이터 개수
        */
        int data_count() const;

        /**
        * 링버퍼의 크기(유지할 최대 데이터 개수)를 반환
        * @return 링버퍼 크기
        */
        int buffer_size() const;

        /**
        * 링버퍼의 크기를 입력 size로 변경
        * @param size 링버퍼 최대 크기 (유지할 데이터의 개수)
        */
        bool resize(int size);

        /**
        * 링버퍼의 첫번째 데이터를 반환
        * @return 링버퍼에 저장된 첫 번째 데이터
        */
        T& front();

        /**
        * 링버퍼의 첫번째 데이터를 반환
        * @return 링버퍼에 저장된 첫 번째 데이터
        */
        const T& front() const;

        /**
        * 링버퍼의 마지막 번째 데이터를 반환
        * @return 링버퍼에 저장된 마지막 번째 데이터
        */
        T& back();

        /**
        * 링버퍼의 마지막 번째 데이터를 반환
        * @return 링버퍼에 저장된 마지막 번째 데이터
        */
        const T& back() const;

        /**
        * 연산자 중첩 (가장 오래된 순으로 i번째 데이터 반환)
        * @return 링버퍼에 저장된 최근 i번째 데이터
        */
        T& operator [](int i);

        /**
        * 연산자 중첩 (가장 오래된 순으로 i번째 데이터 반환)
        * @return 링버퍼에 저장된 최근 i번째 데이터
        */
        const T operator [](int i) const;

    protected:
        /** Get internal buffer index of a given external index */
        int internal_index(int index) const { return (m_next_index - m_data_count + index + m_buf_size) % m_buf_size; }

        /** 링버퍼 메모리 */
        T* m_buf;

        /** 링버퍼 최대 크기 */
        int m_buf_size;

        /** 다음에 저장될 데이터의 위치(버퍼 인덱스) */
        int m_next_index;

        /** 현재 저장된 데이터 개수 */
        int m_data_count;
    };

    /**
    * @brief 데이터가 숫자인 링버퍼 구현
    *
    * sum(), average() 등의 숫자 데이터에 특화된 추가 연산 제공
    */
    template <class T>
    class RingBufferNumeric : public RingBuffer<T>
    {
    public:
        /**
        * 기본 생성자 (empty 링버퍼 생성)
        */
        RingBufferNumeric();

        /**
        * 생성자
        * @param size 링버퍼 최대 크기 (유지할 데이터의 개수)
        */
        RingBufferNumeric(int size);

        /**
        * 데이터를 링버퍼 끝에 삽입 (버퍼 최대크기에 도달할 경우 가장 오래된 데이터를 제거)
        * @param data 추가할 데이터
        */
        bool push_back(const T& data);

        /**
        * 데이터를 링버퍼 중간에 삽입 (버퍼 최대크기에 도달할 경우 가장 오래된 데이터를 제거)
        * @param data 삽입할 데이터
        * @return Index of newly inserted data if succeeds (return -1 if failed)
        */
        virtual int insert(int index, const T& data);

        /**
        * 데이터 구간을 삭제
        * @param first_idx Index of first element to erase
        * @param last_idx Index of last element to erase (if it is given -1, erase to the end)
        * @return Return True if successful
        */
        virtual bool erase(int first_idx, int last_idx = -1);

        /**
        * 현재 버퍼에 저장된 데이터 합을 반환
        * @return 데이터 합
        */
        T sum();

        /**
        * 현재 버퍼에 저장된 데이터 평균을 반환
        * @return 데이터 평균
        */
        double average();

    protected:
        /** 링버퍼에 저장된 데이터 합 */
        T m_sum;
    };

    template<class T>
    RingBuffer<T>::RingBuffer()
    {
        m_buf = nullptr;
        m_buf_size = 0;
        m_next_index = 0;
        m_data_count = 0;
    }

    template<class T>
    RingBuffer<T>::RingBuffer(int size)
    {
        assert(size > 0);

        m_buf = new T[size];
        m_buf_size = size;
        m_next_index = 0;
        m_data_count = 0;
    }

    template<class T>
    RingBuffer<T>::~RingBuffer()
    {
        delete[] m_buf;
    }

    template<class T>
    bool RingBuffer<T>::empty() const
    {
        return (m_data_count <= 0);
    }

    template<class T>
    bool RingBuffer<T>::push_back(const T& data)
    {
        if (m_buf == nullptr) return false;

        m_buf[m_next_index] = data;
        m_next_index = (m_next_index + 1) % m_buf_size;
        if (m_data_count < m_buf_size) m_data_count++;
        return true;
    }

    template<class T>
    int RingBuffer<T>::insert(int index, const T& data)
    {
        if (m_buf == nullptr || index == 0 && m_data_count == m_buf_size) return -1;
        if (index < 0 && index >= m_data_count) return -1;

        int index_in = internal_index(index);
        int i = m_next_index;
        while (i != index_in)
        {
            int i_prev = (i - 1 + m_buf_size) % m_buf_size;
            m_buf[i] = m_buf[i_prev];
            i = i_prev;
        }
        m_buf[i] = data;

        m_next_index = (m_next_index + 1) % m_buf_size;
        if (m_data_count < m_buf_size)
        {
            m_data_count++;
            return index;
        }
        return (index - 1);
    }

    template<class T>
    bool RingBuffer<T>::erase(int first, int last)
    {
        if (last < 0) last = m_data_count - 1;
        if (m_buf == nullptr || first<0 || last>=m_data_count || last < first) return false;

        int copy_to = internal_index(first);
        int copy_from = internal_index(last + 1);
        while (copy_from != m_next_index)
        {
            m_buf[copy_to] = m_buf[copy_from];
            copy_to = (copy_to + 1) % m_buf_size;
            copy_from = (copy_from + 1) % m_buf_size;
        }
        m_next_index = copy_to;
        m_data_count -= (last - first + 1);
        if (m_data_count <= 0) m_next_index = 0;
        return true;
    }

    template<class T>
    int RingBuffer<T>::data_count() const
    {
        return m_data_count;
    }

    template<class T>
    int RingBuffer<T>::buffer_size() const
    {
        return m_buf_size;
    }

    template<class T>
    bool RingBuffer<T>::resize(int size)
    {
        if (size < 0) return false;

        if (m_buf) delete[] m_buf;

        m_buf = new T[size];
        m_buf_size = size;
        m_next_index = 0;
        m_data_count = 0;
        return true;
    }

    template<class T>
    T& RingBuffer<T>::front()
    {
        assert(m_data_count > 0);

        int in = internal_index(0);
        return m_buf[in];
    }

    template<class T>
    const T& RingBuffer<T>::front() const
    {
        assert(m_data_count > 0);

        int in = internal_index(0);
        return m_buf[in];
    }

    template<class T>
    T& RingBuffer<T>::back()
    {
        assert(m_data_count > 0);

        int in = (m_next_index - 1 + m_buf_size) % m_buf_size;
        return m_buf[in];
    }

    template<class T>
    const T& RingBuffer<T>::back() const
    {
        assert(m_data_count > 0);

        int in = (m_next_index - 1 + m_buf_size) % m_buf_size;
        return m_buf[in];
    }

    template<class T>
    T& RingBuffer<T>::operator [] (int i)
    {
        assert(i >= 0 && i < m_data_count);

        int in = internal_index(i);
        return m_buf[in];
    }

    template<class T>
    const T RingBuffer<T>::operator [] (int i) const
    {
        assert(i >= 0 && i < m_data_count);

        int in = internal_index(i);
        return m_buf[in];
    }

    template<class T>
    RingBufferNumeric<T>::RingBufferNumeric()
    {
        m_sum = 0;
    }

    template<class T>
    RingBufferNumeric<T>::RingBufferNumeric(int size)
        : RingBuffer<T>(size)
    {
        m_sum = 0;
    }

    template<class T>
    bool RingBufferNumeric<T>::push_back(const T& data)
    {
        if (m_buf == nullptr) return false;
        if (m_data_count == m_buf_size) m_sum -= m_buf[m_next_index];
        m_sum += data;
        RingBuffer<T>::push_back(data);
        return true;
    }

    template<class T>
    int RingBufferNumeric<T>::insert(int index, const T& data)
    {
        if (m_buf == nullptr || index == 0 && m_data_count == m_buf_size) return -1;
        if (m_data_count == m_buf_size) m_sum -= m_buf[m_next_index];
        m_sum += data;
        return RingBuffer<T>::insert(index, data);
    }

    template<class T>
    bool RingBufferNumeric<T>::erase(int first, int last)
    {
        if (last < 0) last = m_data_count - 1;
        if (m_buf == nullptr || first < 0 || last >= m_data_count || last < first) return false;

        for (int i = first; i <= last; i++)
        {
            int in = internal_index(i);
            sum -= m_buf[in];
        }
        return RingBuffer<T>::erase(first, last);
    }

    template<class T>
    T RingBufferNumeric<T>::sum()
    {
        return m_sum;
    }

    template<class T>
    double RingBufferNumeric<T>::average()
    {
        if (m_data_count > 0)
            return m_sum / (double)m_data_count;
        else
            return 0;
    }

    std::vector<std::string> splitStr(const char* buf, int buf_len, char dlm = ',');

} // End of 'dg'

#endif // End of '__DG_UTILS_UTILITY__'
