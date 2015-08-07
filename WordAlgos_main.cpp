

#include <cstdio>
#include <iostream>
#include <string>
#include <vector>
#include <algorithm>

#include "ZAssert.h"

std::string longestNonConsecutiveSubstring(const std::vector<char>& s0, const std::vector<char>& s1)
{
/// Table:
/// In field row = i, column = j, we have longest non-consecutive substring in s0[0..i], s1[0..j]
///
///   \ s1 |  a l a
/// s0 \   |
/// --------------------------------- 
///    a   |  1 1 1
///    a   |  1 1 1
///    l   |  1 2 2
///    a   |  1 2 3
///    b   |  1 2 3
///    c   |  1 2 3
///

    std::vector<int> previousCounts(s1.size());
    std::vector<int> resultCounts(s1.size());

    std::vector<std::string> previousStrings(s1.size());
    std::vector<std::string> resultStrings(s1.size());

    for (size_t i = 0; i < s0.size(); ++i)
    {
        previousCounts.swap(resultCounts);
        previousStrings.swap(resultStrings);
        for (size_t j = 0; j < s1.size(); ++j)
        {
            if (s0[i] == s1[j])
            {
                int previous = 0;
                std::string previousStr;
                if (j > 0)
                {
                    previous = previousCounts[j - 1];
                    previousStr = previousStrings[j - 1];
                }
                resultCounts[j] = previous + 1;
                resultStrings[j] = previousStr + s1[j];
            }
            else
            {
                int last = 0;
                std::string lastStr;
                if (j > 0)
                {
                    last = resultCounts[j - 1];
                    lastStr = resultStrings[j - 1];
                }
                // here we do this: resultCounts[j] = std::max(resultCounts[j - 1], previousCounts[j])
                if (last > previousCounts[j])
                {
                    resultCounts[j] = last;
                    resultStrings[j] = lastStr;
                }
                else
                {
                    resultCounts[j] = previousCounts[j];
                    resultStrings[j] = previousStrings[j];
                }
            }
        }
    }

    if (resultStrings.empty())
        return "";

    return resultStrings.back();
}

std::string longestConsecutiveSubstring(const std::vector<char>& s0, const std::vector<char>& s1)
{
    /// Table:
    /// In field row = i, column = j, we have longest consecutive substring ending in exactly s0[i] and s1[j]
    ///
    ///   \ s1 |  a l a
    /// s0 \   |
    /// --------------------------------- 
    ///    a   |  1 0 1
    ///    a   |  1 0 1
    ///    l   |  0 2 0
    ///    a   |  1 0 1
    ///    b   |  0 0 0
    ///    c   |  0 0 0
    ///

    std::vector<int> previousCounts(s1.size());
    std::vector<int> resultCounts(s1.size());

    std::vector<std::string> previousStrings(s1.size());
    std::vector<std::string> resultStrings(s1.size());

    std::string bestString;

    for (size_t i = 0; i < s0.size(); ++i)
    {
        previousCounts.swap(resultCounts);
        previousStrings.swap(resultStrings);
        for (size_t j = 0; j < s1.size(); ++j)
        {
            if (s0[i] == s1[j])
            {
                int previous = 0;
                std::string previousStr;
                if (j > 0)
                {
                    previous = previousCounts[j - 1];
                    previousStr = previousStrings[j - 1];
                }
                resultCounts[j] = previous + 1;
                resultStrings[j] = previousStr + s1[j];

                if (resultStrings[j].length() > bestString.length())
                {
                    bestString = resultStrings[j];
                }
            }
            else
            {
                resultCounts[j] = 0;
                resultStrings[j].clear();
            }
        }
    }

    return bestString;
}

std::vector<int> prefixSuffixTable(const std::vector<char>& s)
{
    /// Prefix suffix table:
    /// http://wazniak.mimuw.edu.pl/index.php?title=Algorytmy_i_struktury_danych/Algorytmy_tekstowe_I
    ///
    /// P[i] = longest prefix (shorter than the word itself) of word  s[0..i] that is also it's suffix.
    /// So if P[i] == k, then s[0..k-1] == s[n-k..n-1]

    std::vector<int> P(s.size());
    int l = 0;
    P[0] = 0;
    for (int i = 1; i < s.size(); ++i)
    {
        while(true)
        {
            if (s[l] == s[i])
            {
                l++;
                break;
            }
            if (l == 0)
                break;
            l = P[l - 1];
        }
        P[i] = l;
    }

    return P;
}

std::string longestPrefixPalindrome(const std::vector<char>& s)
{
    if (s.empty())
        return "";

    /// Longest prefix palindrome is the longest prefix-suffix in word: s # s-reversed
    std::vector<char> sXs(s);
    sXs.push_back('\0'); // "#"
    for (int i = 0; i < s.size(); ++i)
        sXs.push_back(s[s.size() - 1 - i]);

    std::vector<int> P = prefixSuffixTable(sXs);
    if (P.back() == 0)
        return "";

    std::string result(s.begin(), s.begin() + P.back());
    return result;
}

std::vector<char> addSeparators(const std::vector<char>& s)
{
    std::vector<char> result;
    result.reserve(s.size() * 2 + 1);

    const char separator = '\0';
    result.push_back(separator);
    for (auto c : s)
    {
        result.push_back(c);
        result.push_back(separator);
    }

    return result;
}

/// @brief Result[i] is the radius of odd palindrome centered on i.
///        Result.size() == s.size();
/// @note Manacher's algorithm
///       https://en.wikipedia.org/wiki/Longest_palindromic_substring
std::vector<int> allOddPalindromes(const std::vector<char>& s)
{
    std::vector<int> R(s.size()); // radiuses of all odd palindromes for each position in string
    if (s.empty())
        return std::vector<int>();

    int pos = 0;
    int radius = 1;
    while (pos < s.size())
    {
        // Calculate radius of odd palindrome for position 'pos'
        while ( (radius <= pos) && (pos + radius < s.size()) && (s[pos - radius] == s[pos + radius]) )
            radius++;

        R[pos] = radius;

        // Starting point for next loop. We might change it below.
        int nextPos = pos + radius;
        int nextRadius = 1;

        // Now we can compute radiuses for pos..pos+radius-1
        for (int i = 1; i < radius; ++i)
        {
            // We're computing R[pos + i]
            int k = R[pos - i]; // radius of mirror palindrome "on the other side" of pos

            // if mirror palindrome doesn't extend to, or beyond end of R[pos], we can copy the result
            //
            // ....|..[....].....|.....[....]..|....
            //
            if ((i + k) < radius)
            {
                R[pos + i] = k;
                continue;
            }

            // if it extends beyond R[pos], we know that current palindrome extends only to the end of R[pos] palindrome; not further (otherwise R[pos] would be longer)
            //
            // ..[..|.......].....|.....[.......]|...
            //
            if ((i + k) > radius)
            {
                R[pos + i] = radius - i;
                continue;
            }

            // if it ends exactly at the border of R[pos], then current palindrome is at least as long as mirror palindrome; we will try to extend it in next iteration of outer loop.
            //
            // ....|[.......].....|.....[.......|..]..
            //
            nextPos = pos + i;
            nextRadius = k;
            break;
        }

        pos = nextPos;
        radius = nextRadius;
    }

    return R;
}

/// @brief Result[i] is the radius of even palindrome centered between i and i+1.
///        Result.size() == s.size() - 1;
std::vector<int> allEvenPalindromes(const std::vector<char>& s)
{
    std::vector<char> withBorders = addSeparators(s);
    std::vector<int> oddPalindromes = allOddPalindromes(withBorders);

    std::vector<int> result(s.size() - 1);
    for (int i = 0; i < s.size() - 1; ++i)  // last element will be always zero
    {
        result[i] = oddPalindromes[i * 2 + 2] / 2;
    }

    return result;
}

/// @brief Result[2*i + 0] is the radius of odd palindrome centered in i.
///        Result[2*i + 1] is the radius of even palindrome centered between i and i+1.
///        Result.size() == s.size() * 2 - 1;
std::vector<int> allPalindromes(const std::vector<char>& s)
{
    std::vector<char> withBorders = addSeparators(s);
    std::vector<int> oddPalindromes = allOddPalindromes(withBorders);

    std::vector<int> result(s.size() * 2 - 1);
    for (int i = 0; i < s.size() * 2 - 1; ++i)
    {
        result[i] = oddPalindromes[i + 1] / 2;
    }

    return result;
}

void checkPalindromes(const std::vector<int>& odd, const std::vector<int>& even, const std::vector<int>& all)
{
    assert(odd.size() == even.size() + 1);
    assert(all.size() == even.size() * 2 + 1);
    for (int i = 0; i < odd.size(); ++i)
    {
        assert(all[i * 2] == odd[i]);
    }
    for (int i = 0; i < even.size(); ++i)
    {
        assert(all[i * 2 + 1] == even[i]);
    }
}

void WordAlgos_main()
{
    /*
    The freopen function below opens input.txt in read only mode and
    sets your standard input to work with the opened file.
    When you test your code with the sample data, you can use the function
    below to read in from the sample data file instead of the standard input.
    So. you can uncomment the following line for your local test. But you
    have to comment the following line when you submit for your scores.
    */

#ifdef ZBYL
    freopen("words.txt", "r", stdin);
#endif

    int W; // word count

    std::cin >> W;

    for (int i = 0; i < W; ++i)
    {
        std::string s0, s1;
        std::cin >> s0 >> s1;

        std::vector<char> v0(s0.c_str(), s0.c_str() + s0.length());
        std::vector<char> v1(s1.c_str(), s1.c_str() + s1.length());

        auto nconsSubst = longestNonConsecutiveSubstring(v0, v1);
        std::cout << "longestNonConsecutiveSubstring : " << nconsSubst << std::endl;

        auto consSubst = longestConsecutiveSubstring(v0, v1);
        std::cout << "longestConsecutiveSubstring : " << consSubst << std::endl;

        auto P = prefixSuffixTable(v0);
        std::cout << "prefixSuffixTable : " << s0 << std::endl;
        for (auto v : P)
        {
            std::cout << v << " ";
        }
        std::cout << std::endl;

        auto prefPal = longestPrefixPalindrome(v0);
        std::cout << "longestPrefixPalindrome : " << prefPal << std::endl;

        auto oddPals = allOddPalindromes(v0);
        auto evenPals = allEvenPalindromes(v0);
        auto allPals = allPalindromes(v0);
        checkPalindromes(oddPals, evenPals, allPals);
        std::cout << "Palindromes : " << s0 << std::endl;
        std::cout << "Odd: ";
        for (auto v : oddPals)
        {
            std::cout << v << " ";
        }
        std::cout << std::endl;
        std::cout << "Even: ";
        for (auto v : evenPals)
        {
            std::cout << v << " ";
        }
        std::cout << std::endl;
        std::cout << "All:  ";
        for (auto v : allPals)
        {
            std::cout << v << " ";
        }
        std::cout << std::endl;
    }
}
