#ifndef XMLPlanLoaderTest
#define XMLPlanLoaderTest

#include <cppunit/TestCase.h>
#include <cppunit/TestFixture.h>
#include <string>

class MathTest : public CppUnit::TestFixture {
 protected:
   int m_value1, m_value2;

 public:
   MathTest() {}

   void setUp () {
     m_value1 = 2;
     m_value2 = 3;
   }
   
    void testAdd () {
     int result = m_value1 + m_value2;
     CPPUNIT_ASSERT( (2+2) == 5 );
   }

 };


#endif
