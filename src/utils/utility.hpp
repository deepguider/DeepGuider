#ifndef __DG_UTILS_UTILITY__
#define __DG_UTILS_UTILITY__

#include <vector>
#include <string>
#include <assert.h>

namespace dg
{

    /**
     * @brief FIFO(First In First Out) ������ ���� by jylee
     *
     * ���� �ֱ��� n���� �����͸��� �����ϱ� ���� �ڷᱸ�� (���ø� ����)
     */
    template <class T>
    class RingBuffer
    {
    public:
        /**
        * �⺻ ������ (empty ������ ����)
        */
        RingBuffer();

        /**
        * ������
        * @param size ������ �ִ� ũ�� (������ �������� ����)
        */
        RingBuffer(int size);

        /** �Ҹ��� */
        virtual ~RingBuffer();

        /**
        * �������� ������ ���� ���θ� ��ȯ
        * @return Return True if the buffer is empty
        */
        bool empty() const;

        /**
        * �����͸� ������ ���� ���� (���� �ִ�ũ�⿡ ������ ��� ���� ������ �����͸� ����)
        * @param data �߰��� ������
        */
        virtual bool push_back(const T& data);

        /**
        * �����͸� ������ �߰��� ���� (���� �ִ�ũ�⿡ ������ ��� ���� ������ �����͸� ����)
        * @param data ������ ������
        * @return Index of newly inserted data if succeeds (return -1 if failed)
        */
        virtual int insert(int index, const T& data);

        /**
        * ������ ������ ����
        * @param first_idx Index of first element to erase
        * @param last_idx Index of last element to erase (if it is given -1, erase to the end)
        * @return Return True if successful
        */
        virtual bool erase(int first_idx, int last_idx = -1);

        /**
        * �������� ������ ����(���� ����� ������ ����)�� ��ȯ
        * @return ������ ������ ����
        */
        int data_count() const;

        /**
        * �������� ũ��(������ �ִ� ������ ����)�� ��ȯ
        * @return ������ ũ��
        */
        int buffer_size() const;

        /**
        * �������� ũ�⸦ �Է� size�� ����
        * @param size ������ �ִ� ũ�� (������ �������� ����)
        */
        bool resize(int size);

        /**
        * �������� ù��° �����͸� ��ȯ
        * @return �����ۿ� ����� ù ��° ������
        */
        T& front();

        /**
        * �������� ù��° �����͸� ��ȯ
        * @return �����ۿ� ����� ù ��° ������
        */
        const T& front() const;

        /**
        * �������� ������ ��° �����͸� ��ȯ
        * @return �����ۿ� ����� ������ ��° ������
        */
        T& back();

        /**
        * �������� ������ ��° �����͸� ��ȯ
        * @return �����ۿ� ����� ������ ��° ������
        */
        const T& back() const;

        /**
        * ������ ��ø (���� ������ ������ i��° ������ ��ȯ)
        * @return �����ۿ� ����� �ֱ� i��° ������
        */
        T& operator [](int i);

        /**
        * ������ ��ø (���� ������ ������ i��° ������ ��ȯ)
        * @return �����ۿ� ����� �ֱ� i��° ������
        */
        const T operator [](int i) const;

    protected:
        /** Get internal buffer index of a given external index */
        int internal_index(int index) const { return (m_next_index - m_data_count + index + m_buf_size) % m_buf_size; }

        /** ������ �޸� */
        T* m_buf;

        /** ������ �ִ� ũ�� */
        int m_buf_size;

        /** ������ ����� �������� ��ġ(���� �ε���) */
        int m_next_index;

        /** ���� ����� ������ ���� */
        int m_data_count;
    };

    /**
    * @brief �����Ͱ� ������ ������ ����
    *
    * sum(), average() ���� ���� �����Ϳ� Ưȭ�� �߰� ���� ����
    */
    template <class T>
    class RingBufferNumeric : public RingBuffer<T>
    {
    public:
        /**
        * �⺻ ������ (empty ������ ����)
        */
        RingBufferNumeric();

        /**
        * ������
        * @param size ������ �ִ� ũ�� (������ �������� ����)
        */
        RingBufferNumeric(int size);

        /**
        * �����͸� ������ ���� ���� (���� �ִ�ũ�⿡ ������ ��� ���� ������ �����͸� ����)
        * @param data �߰��� ������
        */
        bool push_back(const T& data);

        /**
        * �����͸� ������ �߰��� ���� (���� �ִ�ũ�⿡ ������ ��� ���� ������ �����͸� ����)
        * @param data ������ ������
        * @return Index of newly inserted data if succeeds (return -1 if failed)
        */
        virtual int insert(int index, const T& data);

        /**
        * ������ ������ ����
        * @param first_idx Index of first element to erase
        * @param last_idx Index of last element to erase (if it is given -1, erase to the end)
        * @return Return True if successful
        */
        virtual bool erase(int first_idx, int last_idx = -1);

        /**
        * ���� ���ۿ� ����� ������ ���� ��ȯ
        * @return ������ ��
        */
        T sum();

        /**
        * ���� ���ۿ� ����� ������ ����� ��ȯ
        * @return ������ ���
        */
        double average();

    protected:
        /** �����ۿ� ����� ������ �� */
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
