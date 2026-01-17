#pragma once

#include "jni.h"
#include <memory>

// java.lang.Object

#define JAVA_LANG_OBJECT_SIGNATURE "java/lang/Object"

// java.lang.Integer

#define JAVA_LANG_INTEGER_SIGNATURE "java/lang/Integer"

bool LoadIntegerClass(JNIEnv* env);
void UnloadIntegerClass(JNIEnv* env);

jobject IntegerConstruct(JNIEnv* env, jint value);

// java.lang.Double

#define JAVA_LANG_DOUBLE_SIGNATURE "java/lang/Double"

bool LoadDoubleClass(JNIEnv* env);
void UnloadDoubleClass(JNIEnv* env);

jobject DoubleConstruct(JNIEnv* env, jdouble value);

// java.lang.String

#define JAVA_LANG_STRING_SIGNATURE "java/lang/String"

// java.util.ArrayList

#define JAVA_UTIL_ARRAYLIST_SIGNATURE "java/util/ArrayList"

bool LoadArrayListClass(JNIEnv* env);
void UnloadArrayListClass(JNIEnv* env);

jobject ArrayListConstruct(JNIEnv* env);
jboolean ArrayListAdd(JNIEnv* env, jobject arrayList, jobject element);

// java.util.HashSet

#define JAVA_UTIL_HASHSET_SIGNATURE "java/util/HashSet"

bool LoadHashSetClass(JNIEnv* env);
void UnloadHashSetClass(JNIEnv* env);

jobject HashSetConstruct(JNIEnv* env);
jboolean HashSetAdd(JNIEnv* env, jobject hashSet, jobject element);

// java.util.HashMap

#define JAVA_UTIL_HASHMAP_SIGNATURE "java/util/HashMap"

bool LoadHashMapClass(JNIEnv* env);
void UnloadHashMapClass(JNIEnv* env);

jobject HashMapConstruct(JNIEnv* env);
jobject HashMapPut(JNIEnv* env, jobject hashMap, jobject key, jobject value);

// java.lang.Runnable

#define JAVA_LANG_RUNNABLE_SIGNATURE "java/lang/Runnable"

// We don't need to make runnables right now, just run existing ones.

class RunnableWrapper {
  JNIEnv* m_env = nullptr;
  jobject m_runnable = nullptr;

  jclass m_runnableClass = nullptr;
  jmethodID m_runMethod = nullptr;

 public:
  RunnableWrapper(JNIEnv* env, jobject runnable);
  ~RunnableWrapper();

  JNIEnv* getEnv() const { return m_env; }
  void run();
};

// java.util.function.Consumer<T>

#define JAVA_UTIL_FUNCTION_CONSUMER_SIGNATURE "java/util/function/Consumer"

// We don't need to make consumers right now, just accept existing ones.

class ConsumerWrapper {
  JNIEnv* m_env = nullptr;
  jobject m_consumer = nullptr;

  jclass m_consumerClass = nullptr;
  jmethodID m_acceptMethod = nullptr;

 public:
  ConsumerWrapper(JNIEnv* env, jobject consumer);
  ~ConsumerWrapper();

  JNIEnv* getEnv() const { return m_env; }
  void accept(jobject obj);
};
