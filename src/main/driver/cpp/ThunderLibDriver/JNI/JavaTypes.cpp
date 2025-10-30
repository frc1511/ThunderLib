#include "JavaTypes.hpp"
#include <cstdarg>

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

bool LoadIntegerClass(JNIEnv* env) {
  jclass integerClass = env->FindClass(JAVA_LANG_INTEGER_SIGNATURE);
  if (!integerClass)
    return false;

  s_integerClass = static_cast<jclass>(env->NewGlobalRef(integerClass));

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

RunnableWrapper::RunnableWrapper(JNIEnv* env, jobject runnable) {
  m_env = env;
  m_runnable = env->NewGlobalRef(runnable);
}

RunnableWrapper::~RunnableWrapper() {
  if (m_env && m_runnable) {
    m_env->DeleteGlobalRef(m_runnable);
    m_runnable = nullptr;
  }
}

void RunnableWrapper::run() {
  if (!m_env || !m_runnable)
    return;

  jclass runnableClass = m_env->GetObjectClass(m_runnable);
  jmethodID runMethod = m_env->GetMethodID(runnableClass, "run", "()V");

  if (runMethod) {
    m_env->CallVoidMethod(m_runnable, runMethod);
  }

  m_env->DeleteLocalRef(runnableClass);
}
