#include "JavaTypes.hpp"
#include <cstdarg>
#include <ThunderLibDriver/Logger.hpp>

using thunder::ThunderLibLogger;

static jclass s_integerClass = nullptr;
static jmethodID s_integerConstructor = nullptr;

static jclass s_doubleClass = nullptr;
static jmethodID s_doubleConstructor = nullptr;

static jclass s_arrayListClass = nullptr;
static jmethodID s_arrayListConstructor = nullptr;
static jmethodID s_arrayListAddMethod = nullptr;

static jclass s_hashSetClass = nullptr;
static jmethodID s_hashSetConstructor = nullptr;
static jmethodID s_hashSetAddMethod = nullptr;

static jclass s_hashMapClass = nullptr;
static jmethodID s_hashMapConstructor = nullptr;
static jmethodID s_hashMapPutMethod = nullptr;

bool LoadIntegerClass(JNIEnv* env) {
  jclass integerClass = env->FindClass(JAVA_LANG_INTEGER_SIGNATURE);
  if (!integerClass)
    return false;

  s_integerClass = static_cast<jclass>(env->NewGlobalRef(integerClass));

  env->DeleteLocalRef(integerClass);

  // new Integer(int value)
  s_integerConstructor = env->GetMethodID(s_integerClass, "<init>", "(I)V");
  if (!s_integerConstructor)
    return false;

  return true;
}

void UnloadIntegerClass(JNIEnv* env) {
  if (s_integerClass) {
    env->DeleteGlobalRef(s_integerClass);
    s_integerClass = nullptr;
    s_integerConstructor = nullptr;
  }
}

jobject IntegerConstruct(JNIEnv* env, jint value) {
  jobject integerObject = env->NewObject(s_integerClass, s_integerConstructor, value);
  return integerObject;
}

bool LoadDoubleClass(JNIEnv* env) {
  jclass doubleClass = env->FindClass(JAVA_LANG_DOUBLE_SIGNATURE);
  if (!doubleClass)
    return false;

  s_doubleClass = static_cast<jclass>(env->NewGlobalRef(doubleClass));

  env->DeleteLocalRef(doubleClass);

  // new Double(double value)
  s_doubleConstructor = env->GetMethodID(s_doubleClass, "<init>", "(D)V");
  if (!s_doubleConstructor)
    return false;

  return true;
}

void UnloadDoubleClass(JNIEnv* env) {
  if (s_doubleClass) {
    env->DeleteGlobalRef(s_doubleClass);
    s_doubleClass = nullptr;
    s_doubleConstructor = nullptr;
  }
}

jobject DoubleConstruct(JNIEnv* env, jdouble value) {
  jobject doubleObject = env->NewObject(s_doubleClass, s_doubleConstructor, value);
  return doubleObject;
}

bool LoadArrayListClass(JNIEnv* env) {
  jclass arrayListClass = env->FindClass(JAVA_UTIL_ARRAYLIST_SIGNATURE);
  if (!arrayListClass)
    return false;

  s_arrayListClass = static_cast<jclass>(env->NewGlobalRef(arrayListClass));

  env->DeleteLocalRef(arrayListClass);

  // new ArrayList<>()
  s_arrayListConstructor = env->GetMethodID(s_arrayListClass, "<init>", "()V");
  if (!s_arrayListConstructor)
    return false;

  // boolean add(Object o)
  s_arrayListAddMethod = env->GetMethodID(s_arrayListClass, "add", "(L" JAVA_LANG_OBJECT_SIGNATURE ";)Z");
  if (!s_arrayListAddMethod)
    return false;

  return true;
}

void UnloadArrayListClass(JNIEnv* env) {
  if (s_arrayListClass) {
    env->DeleteGlobalRef(s_arrayListClass);
    s_arrayListClass = nullptr;
    s_arrayListConstructor = nullptr;
    s_arrayListAddMethod = nullptr;
  }
}

jobject ArrayListConstruct(JNIEnv* env) {
  jobject arrayList = env->NewObject(s_arrayListClass, s_arrayListConstructor);
  return arrayList;
}

jboolean ArrayListAdd(JNIEnv* env, jobject arrayList, jobject element) {
  jboolean result = env->CallBooleanMethod(arrayList, s_arrayListAddMethod, element);
  return result;
}

bool LoadHashSetClass(JNIEnv* env) {
  jclass hashSetClass = env->FindClass(JAVA_UTIL_HASHSET_SIGNATURE);
  if (!hashSetClass)
    return false;

  s_hashSetClass = static_cast<jclass>(env->NewGlobalRef(hashSetClass));

  env->DeleteLocalRef(hashSetClass);

  // new HashSet<>()
  s_hashSetConstructor = env->GetMethodID(s_hashSetClass, "<init>", "()V");
  if (!s_hashSetConstructor)
    return false;

  // boolean add(Object o)
  s_hashSetAddMethod = env->GetMethodID(s_hashSetClass, "add", "(L" JAVA_LANG_OBJECT_SIGNATURE ";)Z");
  if (!s_hashSetAddMethod)
    return false;

  return true;
}

void UnloadHashSetClass(JNIEnv* env) {
  if (s_hashSetClass) {
    env->DeleteGlobalRef(s_hashSetClass);
    s_hashSetClass = nullptr;
    s_hashSetConstructor = nullptr;
    s_hashSetAddMethod = nullptr;
  }
}

jobject HashSetConstruct(JNIEnv* env) {
  jobject hashSet = env->NewObject(s_hashSetClass, s_hashSetConstructor);
  return hashSet;
}

jboolean HashSetAdd(JNIEnv* env, jobject hashSet, jobject element) {
  jboolean result = env->CallBooleanMethod(hashSet, s_hashSetAddMethod, element);
  return result;
}

bool LoadHashMapClass(JNIEnv* env) {
  jclass hashMapClass = env->FindClass(JAVA_UTIL_HASHMAP_SIGNATURE);
  if (!hashMapClass)
    return false;

  s_hashMapClass = static_cast<jclass>(env->NewGlobalRef(hashMapClass));

  env->DeleteLocalRef(hashMapClass);

  // new HashMap<>()
  s_hashMapConstructor = env->GetMethodID(s_hashMapClass, "<init>", "()V");
  if (!s_hashMapConstructor)
    return false;

  // Object put(Object key, Object value)
  s_hashMapPutMethod = env->GetMethodID(s_hashMapClass, "put",
                                        "(L" JAVA_LANG_OBJECT_SIGNATURE ";L" JAVA_LANG_OBJECT_SIGNATURE
                                        ";)L" JAVA_LANG_OBJECT_SIGNATURE ";");
  if (!s_hashMapPutMethod)
    return false;

  return true;
}

void UnloadHashMapClass(JNIEnv* env) {
  if (s_hashMapClass) {
    env->DeleteGlobalRef(s_hashMapClass);
    s_hashMapClass = nullptr;
    s_hashMapConstructor = nullptr;
    s_hashMapPutMethod = nullptr;
  }
}

jobject HashMapConstruct(JNIEnv* env) {
  jobject hashMap = env->NewObject(s_hashMapClass, s_hashMapConstructor);
  return hashMap;
}

jobject HashMapPut(JNIEnv* env, jobject hashMap, jobject key, jobject value) {
  jobject previousValue = env->CallObjectMethod(hashMap, s_hashMapPutMethod, key, value);
  return previousValue;
}

RunnableWrapper::RunnableWrapper(JNIEnv* env, jobject runnable) {
  m_env = env;
  m_runnable = env->NewGlobalRef(runnable);

  jclass runnableClass = m_env->GetObjectClass(m_runnable);
  m_runnableClass = static_cast<jclass>(m_env->NewGlobalRef(runnableClass));
  if (!m_runnableClass) {
    ThunderLibLogger::Error("[JNI RunnableWrapper] Failed to get Runnable class");
    return;
  }

  m_env->DeleteLocalRef(runnableClass);

  m_runMethod = m_env->GetMethodID(m_runnableClass, "run", "()V");
  if (!m_runMethod) {
    ThunderLibLogger::Error("[JNI RunnableWrapper] Failed to get Runnable.run() method ID");
  }
}

RunnableWrapper::~RunnableWrapper() {
  if (m_env) {
    if (m_runnable) {
      m_env->DeleteGlobalRef(m_runnable);
      m_runnable = nullptr;
    }
    if (m_runnableClass) {
      m_env->DeleteGlobalRef(m_runnableClass);
      m_runnableClass = nullptr;
    }
  }
}

void RunnableWrapper::run() {
  if (!m_env || !m_runnable || !m_runMethod)
    return;

  m_env->CallVoidMethod(m_runnable, m_runMethod);
}

ConsumerWrapper::ConsumerWrapper(JNIEnv* env, jobject consumer) {
  m_env = env;
  m_consumer = env->NewGlobalRef(consumer);

  jclass consumerClass = m_env->GetObjectClass(m_consumer);
  m_consumerClass = static_cast<jclass>(m_env->NewGlobalRef(consumerClass));
  if (!m_consumerClass) {
    ThunderLibLogger::Error("[JNI ConsumerWrapper] Failed to get Consumer class");
    return;
  }

  m_env->DeleteLocalRef(consumerClass);

  m_acceptMethod = m_env->GetMethodID(m_consumerClass, "accept", "(L" JAVA_LANG_OBJECT_SIGNATURE ";)V");
  if (!m_acceptMethod) {
    ThunderLibLogger::Error("[JNI ConsumerWrapper] Failed to get Consumer.accept() method ID");
  }
}

ConsumerWrapper::~ConsumerWrapper() {
  if (m_env) {
    if (m_consumer) {
      m_env->DeleteGlobalRef(m_consumer);
      m_consumer = nullptr;
    }
    if (m_consumerClass) {
      m_env->DeleteGlobalRef(m_consumerClass);
      m_consumerClass = nullptr;
    }
  }
}

void ConsumerWrapper::accept(jobject obj) {
  if (!m_env || !m_consumer || !m_acceptMethod)
    return;

  m_env->CallVoidMethod(m_consumer, m_acceptMethod, obj);
}
