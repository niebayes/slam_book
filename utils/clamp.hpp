//@cf. Impl of std::clamp (C++17)
// https://en.cppreference.com/w/cpp/algorithm/clamp

template<class T>
constexpr const T& clamp( const T& v, const T& lo, const T& hi )
{
    //! C++14
    // assert( !(hi < lo) );
    return (v < lo) ? lo : (hi < v) ? hi : v;
}

template<class T, class Compare>
constexpr const T& clamp( const T& v, const T& lo, const T& hi, Compare comp )
{
    //! C++14
    // assert( !comp(hi, lo) );
    return comp(v, lo) ? lo : comp(hi, v) ? hi : v;
}