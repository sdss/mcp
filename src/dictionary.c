/*****************************************************************************/
/*
 * A dictionary of keywords for the SDSS-III APO environment
 */
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "as2.h"

/*
 * Define commands
 */
void
loadKeywordDictionary(void)
{
   int i;
   #include "dictionaryData.c"
   
   for (i = 0; keys[i].name != NULL; ++i) {
      declareKeyword(keys[i].name, keys[i].type, keys[i].alwaysSend, keys[i].help);
   }
}

