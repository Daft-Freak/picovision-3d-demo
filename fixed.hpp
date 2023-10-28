#include <cstdint>

template<class T, int frac_bits = sizeof(T) * 4>
class Fixed final
{
public:
    using SameFixed = Fixed<T, frac_bits>;

    // conversions to fixed
    constexpr Fixed(int int_val) : val(int_val << frac_bits) {}
    constexpr Fixed(T int_val) : val(int_val << frac_bits) {}
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
    constexpr explicit operator int() {return val >> frac_bits;}
    constexpr explicit operator T() {return val >> frac_bits;}
    constexpr explicit operator float() {return static_cast<float>(val) / (1 << frac_bits);}

    // basic operators
    constexpr SameFixed &operator +=(SameFixed f) {this->val += f.val; return *this;}
    constexpr SameFixed &operator -=(SameFixed f) {this->val -= f.val; return *this;}

    constexpr SameFixed operator -()
    {
        auto res = *this;
        res.val = -res.val;
        return res;
    }

    // multiply
    constexpr SameFixed &operator *=(SameFixed f)
    {
        // cast to larger type then shift back down
        if constexpr(sizeof(T) >= 4)
            this->val = (static_cast<int64_t>(this->val) * f.val) >> frac_bits;
        else
            this->val = (static_cast<int32_t>(this->val) * f.val) >> frac_bits;
        return *this;
    }

    constexpr SameFixed &operator *=(int i){this->val *= i; return *this;}

    // divide
    constexpr SameFixed &operator /=(SameFixed f)
    {
        // cast to larger type and shift up
        if constexpr(sizeof(T) >= 4)
            this->val = (static_cast<int64_t>(this->val) << frac_bits) / f.val;
        else
            this->val = (static_cast<int32_t>(this->val) << frac_bits) / f.val;
        return *this;
    }

    // TODO: shortcut if only int result wanted

    constexpr SameFixed &operator /=(int i){this->val /= i; return *this;}


    // more operators/wrappers
    constexpr SameFixed operator +(SameFixed f)
    {
        auto res = *this;
        res += f;
        return res;
    }

    constexpr SameFixed operator -(SameFixed f)
    {
        auto res = *this;
        res -= f;
        return res;
    }

    constexpr SameFixed operator *(SameFixed f)
    {
        auto res = *this;
        res *= f;
        return res;
    }

    constexpr SameFixed operator /(SameFixed f)
    {
        auto res = *this;
        res /= f;
        return res;
    }

    inline T raw() const {return val;}

private:
    T val;
};