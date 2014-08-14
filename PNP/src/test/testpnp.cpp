#include "testpnp.h"

#include <cppunit/extensions/TestFactoryRegistry.h>
#include <cppunit/ui/text/TestRunner.h>


namespace learnpnp {

void testPnp() {
CppUnit::Test *suite = CppUnit::TestFactoryRegistry::getRegistry(registryName()).makeTest();
CppUnit::TextUi::TestRunner runner;
runner.addTest( suite );
runner.run();

}

std::string registryName() {
	return "learnpnp";
}

}
