#ifndef __DG_UTILS_UTILITY__
#define __DG_UTILS_UTILITY__

#include <vector>
#include <string>
#include <assert.h>

namespace dg
{

    /**
     * @brief FIFO(First In First Out) 링버퍼 구현
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
        * 데이터를 링버퍼 끝에 삽입 (버퍼 최대크기에 도달할 경우 가장 오래된 데이터를 제거)
        * @param data 추가할 데이터
        */
        virtual void push_back(const T& data);

        /**
        * 링버퍼의 데이터 개수(현재 저장된 데이터 개수)를 반환
        * @return 링버퍼 데이터 개수
        */
        int data_count() const;

        /**
        * 링버퍼의 크기(유지할 최대 데이터 개수)를 반환
        * @return 링버퍼 크기
        */
        int size() const;

        /**
        * 링버퍼의 크기를 입력 size로 변경
        * @param size 링버퍼 최대 크기 (유지할 데이터의 개수)
        */
        void resize(int size);

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
        void push_back(const T& data);

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
        m_buf = NULL;
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
    void RingBuffer<T>::push_back(const T& data)
    {
        assert(m_buf);

        m_buf[m_next_index] = data;
        m_next_index = (m_next_index + 1) % m_buf_size;
        if (m_data_count < m_buf_size) m_data_count++;
    }

    template<class T>
    int RingBuffer<T>::data_count() const
    {
        return m_data_count;
    }

    template<class T>
    int RingBuffer<T>::size() const
    {
        return m_buf_size;
    }

    template<class T>
    void RingBuffer<T>::resize(int size)
    {
        assert(size > 0);

        if (m_buf) delete[] m_buf;

        m_buf = new T[size];
        m_buf_size = size;
        m_next_index = 0;
        m_data_count = 0;
    }

    template<class T>
    T& RingBuffer<T>::operator [] (int i)
    {
        assert(i >= 0 && i < m_data_count);

        int index = (m_next_index + i) % m_buf_size;
        if (m_data_count < m_buf_size) index = i;
        return m_buf[index];
    }

    template<class T>
    const T RingBuffer<T>::operator [] (int i) const
    {
        assert(i >= 0 && i < m_data_count);

        int index = (m_next_index + i) % m_buf_size;
        if (m_data_count < m_buf_size) index = i;
        return m_buf[index];
    }

    template<class T>
    RingBufferNumeric<T>::RingBufferNumeric()
        : RingBuffer<T>()
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
    void RingBufferNumeric<T>::push_back(const T& data)
    {
        if (RingBuffer<T>::m_data_count == RingBuffer<T>::m_buf_size) m_sum -= RingBuffer<T>::m_buf[RingBuffer<T>::m_next_index];
        m_sum += data;
        RingBuffer<T>::push_back(data);
    }

    template<class T>
    T RingBufferNumeric<T>::sum()
    {
        return m_sum;
    }

    template<class T>
    double RingBufferNumeric<T>::average()
    {
        if (RingBuffer<T>::m_data_count > 0)
            return m_sum / (double)RingBuffer<T>::m_data_count;
        else
            return 0;
    }

    std::vector<std::string> splitStr(const char* buf, int buf_len, char dlm = ',');

} // End of 'dg'

#endif // End of '__DG_UTILS_UTILITY__'
