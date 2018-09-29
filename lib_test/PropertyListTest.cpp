#include "stdafx.h"
#include "CppUnitTest.h"

#include "PropertyList.hpp"

using namespace Microsoft::VisualStudio::CppUnitTestFramework;

namespace lib_test
{		
	TEST_CLASS(PropertyListTest)
	{
	public:
		
		TEST_METHOD(canCreateSinglePropertyList)
		{
			PropertyList<float> singlePropertyList;

		}

	};
}