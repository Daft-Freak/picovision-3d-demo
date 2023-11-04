#include <cstdint>
#include <type_traits>

template<class T, int frac_bits = sizeof(T) * 4>
class Fixed final
{
public:
    constexpr Fixed() = default;

    // conversions to fixed
    constexpr Fixed(T int_val) : val(int_val << frac_bits) {}
    template<class U, std::enable_if_t<std::is_integral_v<U>, bool> = true>
    constexpr Fixed(U int_val) : val(int_val << frac_bits) {}
    constexpr Fixed(float float_val) : val(float_val * (1 << frac_bits)) {}

    template<class T2, int frac_bits2>
    constexpr explicit Fixed(Fixed<T2, frac_bits2> f)
    {
        if constexpr(frac_bits2 > frac_bits)
            val = f.raw() >> (frac_bits2 - frac_bits);
        else
            val = f.raw() << (frac_bits - frac_bits2);
    }

    // conversions from fixed
    constexpr explicit operator T() {return val >> frac_bits;}
    template<class U, std::enable_if_t<std::is_integral_v<U>, bool> = true>
    constexpr explicit operator U() {return val >> frac_bits;}
    constexpr explicit operator float() {return static_cast<float>(val) / (1 << frac_bits);}

    // basic operators
    constexpr Fixed &operator +=(Fixed f) {this->val += f.val; return *this;}
    constexpr Fixed &operator -=(Fixed f) {this->val -= f.val; return *this;}

    constexpr Fixed operator -()
    {
        auto res = *this;
        res.val = -res.val;
        return res;
    }

    // multiply
    constexpr Fixed &operator *=(Fixed f)
    {
        // cast to larger type then shift back down
        if constexpr(sizeof(T) >= 4)
            this->val = (static_cast<int64_t>(this->val) * f.val) >> frac_bits;
        else
            this->val = (static_cast<int32_t>(this->val) * f.val) >> frac_bits;
        return *this;
    }

    constexpr Fixed &operator *=(int i){this->val *= i; return *this;}

    // divide
    constexpr Fixed &operator /=(Fixed f)
    {
        // cast to larger type and shift up
        if constexpr(sizeof(T) >= 4)
            this->val = (static_cast<int64_t>(this->val) << frac_bits) / f.val;
        else
            this->val = (static_cast<int32_t>(this->val) << frac_bits) / f.val;
        return *this;
    }

    // TODO: shortcut if only int result wanted

    constexpr Fixed &operator /=(int i){this->val /= i; return *this;}


    // more operators/wrappers
    constexpr Fixed operator +(Fixed f) const
    {
        auto res = *this;
        res += f;
        return res;
    }

    constexpr Fixed operator -(Fixed f) const
    {
        auto res = *this;
        res -= f;
        return res;
    }

    constexpr Fixed operator *(Fixed f) const
    {
        auto res = *this;
        res *= f;
        return res;
    }

    constexpr Fixed operator *(int i) const
    {
        auto res = *this;
        res *= i;
        return res;
    }

    constexpr Fixed operator /(Fixed f) const
    {
        auto res = *this;
        res /= f;
        return res;
    }

    constexpr Fixed operator /(int i) const
    {
        auto res = *this;
        res /= i;
        return res;
    }

    constexpr Fixed reciprocal() const
    {
        Fixed ret;
        if constexpr(frac_bits >= sizeof(T) * 4)
        {
            // less precise, but faster
            // worse the more fractional bits you have
            constexpr int target_frac_bits = sizeof(T) * 4 - 1;
            constexpr int shift = frac_bits - target_frac_bits;
            ret.val = ((1 << (target_frac_bits * 2)) / (val >> shift)) << shift;
        }
        else // no less precise, still faster
            ret.val = (1 << (frac_bits * 2)) / val;

        return ret;
    }

    inline T raw() const {return val;}

    static Fixed from_raw(T val)
    {
        Fixed ret;
        ret.val = val;

        return ret;
    }

private:
    T val;
};

template<int frac_bits = 16>
using Fixed32 = Fixed<int32_t, frac_bits>;

template<int frac_bits = 16>
using UFixed32 = Fixed<uint32_t, frac_bits>;