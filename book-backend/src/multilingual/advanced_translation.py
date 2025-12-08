"""Advanced translation features with ML and context-aware capabilities."""

import logging
import asyncio
import json
from typing import Dict, Any, List, Optional, Tuple
from datetime import datetime, timedelta
from enum import Enum
from dataclasses import dataclass, asdict
import uuid
import re
from collections import defaultdict
import numpy as np
from sklearn.feature_extraction.text import TfidfVectorizer
from sklearn.metrics.pairwise import cosine_similarity

from sqlalchemy.orm import Session
from sqlalchemy import and_, or_, func, desc

from app.models.multilingual import (
    ContentTranslation, TranslationMemory, TerminologyGlossary,
    TranslationStatus, TranslationMethod, Language
)
from app.models.database import User

logger = logging.getLogger(__name__)


class ConfidenceLevel(str, Enum):
    """Translation confidence levels."""
    VERY_HIGH = "very_high"  # 90-100%
    HIGH = "high"           # 80-89%
    MEDIUM = "medium"       # 60-79%
    LOW = "low"            # 40-59%
    VERY_LOW = "very_low"  # 0-39%


class TranslationDomain(str, Enum):
    """Translation domains for context-aware translation."""
    TECHNICAL = "technical"
    BUSINESS = "business"
    ACADEMIC = "academic"
    CASUAL = "casual"
    LEGAL = "legal"
    MEDICAL = "medical"


@dataclass
class TranslationAlternative:
    """Alternative translation option."""
    text: str
    confidence: float
    source: str  # "ai", "memory", "human"
    domain: TranslationDomain
    metadata: Dict[str, Any]


@dataclass
class ContextualTranslation:
    """Context-aware translation result."""
    primary_translation: str
    confidence: ConfidenceLevel
    alternatives: List[TranslationAlternative]
    domain: TranslationDomain
    context_factors: Dict[str, float]
    fallback_used: bool
    quality_score: float


class AdvancedTranslationSystem:
    """Advanced translation system with ML and context awareness."""
    
    def __init__(self):
        self.domain_models = {}
        self.confidence_thresholds = {
            ConfidenceLevel.VERY_HIGH: 0.9,
            ConfidenceLevel.HIGH: 0.8,
            ConfidenceLevel.MEDIUM: 0.6,
            ConfidenceLevel.LOW: 0.4,
            ConfidenceLevel.VERY_LOW: 0.0
        }
        
        # Domain-specific terminology weights
        self.domain_weights = {
            TranslationDomain.TECHNICAL: {
                "terminology_weight": 0.4,
                "context_weight": 0.3,
                "memory_weight": 0.3
            },
            TranslationDomain.BUSINESS: {
                "terminology_weight": 0.3,
                "context_weight": 0.4,
                "memory_weight": 0.3
            },
            TranslationDomain.ACADEMIC: {
                "terminology_weight": 0.5,
                "context_weight": 0.3,
                "memory_weight": 0.2
            }
        }
        
        # Initialize vectorizer for similarity calculations
        self.vectorizer = TfidfVectorizer(
            max_features=1000,
            stop_words='english',
            ngram_range=(1, 2)
        )
        
        self.translation_memory_cache = {}
        self.terminology_cache = {}
    
    async def translate_with_context(
        self,
        source_text: str,
        source_language: str,
        target_language: str,
        domain: TranslationDomain = TranslationDomain.TECHNICAL,
        context: Optional[str] = None,
        user_preferences: Optional[Dict[str, Any]] = None,
        db: Session = None
    ) -> ContextualTranslation:
        """Perform context-aware translation with confidence scoring."""
        try:
            # Detect domain if not specified
            if domain == TranslationDomain.TECHNICAL:
                domain = await self._detect_domain(source_text)
            
            # Get translation alternatives from multiple sources
            alternatives = await self._get_translation_alternatives(
                source_text, source_language, target_language, domain, context, db
            )
            
            # Score and rank alternatives
            scored_alternatives = await self._score_alternatives(
                alternatives, source_text, domain, context, user_preferences
            )
            
            # Select primary translation
            primary_translation = scored_alternatives[0] if scored_alternatives else None
            
            if not primary_translation:
                # Fallback to basic translation
                fallback_translation = await self._fallback_translation(
                    source_text, source_language, target_language
                )
                return ContextualTranslation(
                    primary_translation=fallback_translation,
                    confidence=ConfidenceLevel.LOW,
                    alternatives=[],
                    domain=domain,
                    context_factors={},
                    fallback_used=True,
                    quality_score=0.3
                )
            
            # Calculate confidence level
            confidence = self._calculate_confidence_level(primary_translation.confidence)
            
            # Calculate context factors
            context_factors = await self._analyze_context_factors(
                source_text, primary_translation.text, domain, context
            )
            
            return ContextualTranslation(
                primary_translation=primary_translation.text,
                confidence=confidence,
                alternatives=scored_alternatives[1:6],  # Top 5 alternatives
                domain=domain,
                context_factors=context_factors,
                fallback_used=False,
                quality_score=primary_translation.confidence
            )
            
        except Exception as e:
            logger.error(f"Error in context-aware translation: {e}")
            # Return fallback translation
            fallback_text = await self._fallback_translation(
                source_text, source_language, target_language
            )
            return ContextualTranslation(
                primary_translation=fallback_text,
                confidence=ConfidenceLevel.VERY_LOW,
                alternatives=[],
                domain=domain,
                context_factors={},
                fallback_used=True,
                quality_score=0.2
            )    

    async def _detect_domain(self, text: str) -> TranslationDomain:
        """Detect the domain of the text using keyword analysis."""
        try:
            # Domain-specific keywords
            domain_keywords = {
                TranslationDomain.TECHNICAL: [
                    'algorithm', 'function', 'variable', 'database', 'server',
                    'programming', 'software', 'hardware', 'network', 'system',
                    'code', 'development', 'framework', 'library', 'api'
                ],
                TranslationDomain.BUSINESS: [
                    'revenue', 'profit', 'market', 'customer', 'sales',
                    'strategy', 'management', 'finance', 'investment', 'budget',
                    'company', 'business', 'corporate', 'enterprise', 'commercial'
                ],
                TranslationDomain.ACADEMIC: [
                    'research', 'study', 'analysis', 'theory', 'hypothesis',
                    'methodology', 'literature', 'academic', 'scholarly', 'peer',
                    'journal', 'publication', 'citation', 'abstract', 'conclusion'
                ]
            }
            
            text_lower = text.lower()
            domain_scores = {}
            
            for domain, keywords in domain_keywords.items():
                score = sum(1 for keyword in keywords if keyword in text_lower)
                domain_scores[domain] = score / len(keywords)
            
            # Return domain with highest score, default to technical
            best_domain = max(domain_scores, key=domain_scores.get)
            return best_domain if domain_scores[best_domain] > 0.1 else TranslationDomain.TECHNICAL
            
        except Exception as e:
            logger.error(f"Error detecting domain: {e}")
            return TranslationDomain.TECHNICAL
    
    async def _get_translation_alternatives(
        self,
        source_text: str,
        source_language: str,
        target_language: str,
        domain: TranslationDomain,
        context: Optional[str],
        db: Session
    ) -> List[TranslationAlternative]:
        """Get translation alternatives from multiple sources."""
        alternatives = []
        
        try:
            # 1. Translation Memory matches
            memory_alternatives = await self._get_memory_alternatives(
                source_text, source_language, target_language, db
            )
            alternatives.extend(memory_alternatives)
            
            # 2. AI Translation with domain-specific models
            ai_alternatives = await self._get_ai_alternatives(
                source_text, source_language, target_language, domain, context
            )
            alternatives.extend(ai_alternatives)
            
            # 3. Human translations (if available)
            human_alternatives = await self._get_human_alternatives(
                source_text, source_language, target_language, db
            )
            alternatives.extend(human_alternatives)
            
            # 4. Terminology-based translations
            terminology_alternatives = await self._get_terminology_alternatives(
                source_text, source_language, target_language, domain, db
            )
            alternatives.extend(terminology_alternatives)
            
            return alternatives
            
        except Exception as e:
            logger.error(f"Error getting translation alternatives: {e}")
            return []
    
    async def _get_memory_alternatives(
        self,
        source_text: str,
        source_language: str,
        target_language: str,
        db: Session
    ) -> List[TranslationAlternative]:
        """Get alternatives from translation memory."""
        try:
            # Search for similar translations in memory
            memory_entries = db.query(TranslationMemory).filter(
                and_(
                    TranslationMemory.source_language == source_language,
                    TranslationMemory.target_language == target_language
                )
            ).limit(100).all()
            
            if not memory_entries:
                return []
            
            # Calculate similarity scores
            source_texts = [entry.source_text for entry in memory_entries]
            source_texts.append(source_text)
            
            try:
                tfidf_matrix = self.vectorizer.fit_transform(source_texts)
                similarity_scores = cosine_similarity(
                    tfidf_matrix[-1:], tfidf_matrix[:-1]
                ).flatten()
            except Exception:
                # Fallback to simple string matching
                similarity_scores = [
                    self._simple_similarity(source_text, entry.source_text)
                    for entry in memory_entries
                ]
            
            alternatives = []
            for i, entry in enumerate(memory_entries):
                if similarity_scores[i] > 0.3:  # Minimum similarity threshold
                    alternatives.append(TranslationAlternative(
                        text=entry.target_text,
                        confidence=similarity_scores[i] * (entry.quality_score or 0.8),
                        source="memory",
                        domain=TranslationDomain.TECHNICAL,  # Default
                        metadata={
                            "similarity": similarity_scores[i],
                            "original_quality": entry.quality_score,
                            "usage_count": entry.usage_count or 1
                        }
                    ))
            
            return sorted(alternatives, key=lambda x: x.confidence, reverse=True)[:3]
            
        except Exception as e:
            logger.error(f"Error getting memory alternatives: {e}")
            return []
    
    def _simple_similarity(self, text1: str, text2: str) -> float:
        """Calculate simple similarity between two texts."""
        words1 = set(text1.lower().split())
        words2 = set(text2.lower().split())
        
        if not words1 or not words2:
            return 0.0
        
        intersection = words1.intersection(words2)
        union = words1.union(words2)
        
        return len(intersection) / len(union)
    
    async def _get_ai_alternatives(
        self,
        source_text: str,
        source_language: str,
        target_language: str,
        domain: TranslationDomain,
        context: Optional[str]
    ) -> List[TranslationAlternative]:
        """Get AI-generated translation alternatives."""
        try:
            # This would integrate with actual AI translation services
            # For now, simulate multiple AI alternatives
            
            alternatives = []
            
            # Simulate different AI models/approaches
            ai_models = [
                {"name": "general", "confidence": 0.85},
                {"name": "domain_specific", "confidence": 0.90},
                {"name": "context_aware", "confidence": 0.88}
            ]
            
            for model in ai_models:
                # In real implementation, this would call actual AI services
                translated_text = await self._simulate_ai_translation(
                    source_text, source_language, target_language, model["name"]
                )
                
                alternatives.append(TranslationAlternative(
                    text=translated_text,
                    confidence=model["confidence"],
                    source="ai",
                    domain=domain,
                    metadata={
                        "model": model["name"],
                        "context_used": context is not None
                    }
                ))
            
            return alternatives
            
        except Exception as e:
            logger.error(f"Error getting AI alternatives: {e}")
            return []
    
    async def _simulate_ai_translation(
        self,
        source_text: str,
        source_language: str,
        target_language: str,
        model_name: str
    ) -> str:
        """Simulate AI translation (replace with actual AI service calls)."""
        # This is a placeholder - in real implementation, integrate with:
        # - Google Translate API
        # - Azure Translator
        # - Custom domain-specific models
       tem()ysonSTranslaticedanion = Adv_translat
advancedm instancetion systeslaanced tran# Global adv


)ory: {e}"nslation memupdating trarror .error(f"E    logger
        tion as e:t Excep    excep
                    )
_score}"ry: {qualitymoon to metiranslagh-quality thidded "Aogger.info(f    l                
           
 mmit()db.co               ry)
 ry_entdd(memo     db.a   
                         )
        1
       unt=  usage_co              
    ",anslationtranced_"advext=    cont              value,
  in=domain.        doma          core,
  ality_score=quty_suali        q          uage,
  nglaage=target_t_languge      tar              guage,
anource_language=ssource_l                   get_text,
 =tarxt  target_te                 ,
 ource_textt=se_texsourc                 mory(
   ationMensltry = Tra   memory_en     
        onsy translatigh-qualittore hi8:  # Only s0.ore >= quality_sc      if ry:
          t"
    s.""lation trans-qualityigh hry with newn memoranslatio t"""Update
        ne:No) -> sion
    b: Ses    d
    n,onDomaiTranslati  domain:   ,
    e: floatorality_scqur,
        uage: stet_lang  targtr,
      _language: s   sourcetr,
     t: sexarget_t,
        ttr: ssource_text        elf,
 s       mory(
n_me_translatio def update    async
"
    ce_text}]ation: {sourback translurn f"[Fallet    r    ceholder
a platurn For now, re      #    service
ionc translath a basintegrate witThis would i     #    ail."""
thods fadvanced meation when lback translide fal """Prov:
       > str
    ) -ge: str_languaarget      tstr,
  uage: ce_langour     st: str,
   ce_tex  sour         self,
 ation(
    ransl_tef _fallbackc d asyn 
   
   0.5  return         
  nce: {e}")n relevang domailculatir caError.error(f"     logge
       s e:ion aceptcept Ex       ex      
    ), 1.0)
   n(keywordstches / leturn min(mare  
                     t_lower)
 yword in texwords if kerd in keyor keywom(1 fatches = su        mr()
    xt.lower = te  text_lowe
            
          elevanceutral rn 0.5  # Neetur      r          eywords:
 kif not            )
main, []words.get(do_keyainrds = dom   keywo         
  
               }
        'theory']s','analysi, ch', 'study'resear['IC: in.ACADEMnDomatioTransla          
      omer'],enue', 'custt', 'rev', 'marke: ['businessin.BUSINESSationDoma    Transl        
    ta'],orithm', 'da, 'alg 'system'l',echnica'tCAL: [in.TECHNIlationDomans   Tra            = {
  _keywords   domain     
    try:      n."""
   domai specific a is toextlevant te how re""Calculat
        ") -> float:omain
    ationDain: Transl dom  
     tr,  text: sf,
              selnce(
elevaomain_rlate_df _calcudec  asyn  
    
 urn {}ret         
   e}")rs: {ntext factoing coalyzan(f"Error rorger.er         log  ion as e:
 ept Exceptexc  
                  rn factors
       retu
             
     = 0.0lignment"]xt_aors["conte        fact        else:
      )
       contextnslation,e(tranceleva_context_rulateself._calcwait  = aignment"]context_altors["  fac            ntext:
  co  if 
          nttext alignme# Con           
            omain)
 slation, dance(tranomain_releve_dcalculatait self._ance"] = awmain_relevctors["do  fa
           relevanceinma      # Do 
          
        1), 2.0)e_sentences,ourcax(s/ ms _sentenceonin(translati"] = meservationty_prmplexis["co   factor)
         anslation)tr', (r'[.!?]+re.splitlen(ences = _sentionanslat         tr   xt))
 source_te+',lit(r'[.!?]spre. = len(sentencesce_ur   so         cture)
ence struased on sent (bfactor Complexity      #
            io
       ength_rato"] = l_ratingth["leors   fact     , 1)
    rce_text) max(len(souon) /en(translati l_ratio =     length
       torLength fac  #          
        try:   
 {}
     ctors =       fa
  ation."""d the translnfluencehat iors tfact"Analyze ""]:
        str, floatict[-> D    ) l[str]
onaxt: Optionte
        cDomain,lationn: Trans    domai: str,
    ationsl      trant: str,
  ource_tex     s
   f,    sels(
    ext_factoranalyze_contdef _nc  asy
       ERY_LOW
vel.VdenceLern Confietu    rel
    n levetur r              d:
 = thresholence_score > confid     ife):
       e=Trux[1], reversa x: key=lambd                                 
    .items(), thresholdsence_confidsorted(self.shold in thre for level, "
       .""dence levelconficore to ence sfidvert con"Con  ""    
  l:evenceL) -> Confide: floatdence_scoref, confilevel(selence_fidlculate_con  def _ca
  
    0.0    return )
        ces: {e}"ser preferenlying u"Error appgger.error(f       lo
     n as e:ept Exceptio
        exc          t
  return boos         
               += 0.08
   boost           omains:
   d_dferree in predomain.valuive.rnat  if alte
          s", [])_domainredt("prefererences.gens = prefrred_domai prefe   ce
        ise preferenn expert     # Domai 
                = 0.05
  ost +       bo
         "):style", "ata.get("ative.metadalternmal" in d "inforaninformal" ty == "ormali   elif f         05
boost += 0.           ""):
     "style", a.get(.metadaternativealt in  "formal"l" andy == "formarmalitf fo i    ")
       "neutral", ormality.get("freferencesity = p    formal
        anguageformal ll/inrma foor freference       # P    
         1
     += 0.     boost          uman":
 ce == "hourternative.s alndse) ahuman", Fal"prefer_ences.get( prefer   if
         ionsatnslhuman traor reference f    # P         try:
       
        
st = 0.0oo  b  ""
    s."onof translatitain types  cer boosts toeferencely user pr"""App
        > float: ) -Any]
   t[str, ences: Dicrefer
        pAlternative,ranslationive: Tatalternf,
        
        selences(_user_preferapply  def _
    
   return 0.0         ")
  ance: {e}elevtext rlating conor calcuErr"ror(fer.ergg   lo  :
       on as excepti    except E  
    
          words))n(context_.unioon_wordstin(translaap) / len(overl return le          t_words)
 ion(contexsect.interordsanslation_woverlap = tr        
              
   return 0.0             t_words:
  t contexwords or non_anslatio  if not tr                   
)
   lit()wer().spntext.lo = set(coxt_wordste       conit())
     .lower().spltionet(transla = son_words   translati      
   tionap calculaeyword overl# Simple k   
               try:
  ."""extn conthe gives to tslation ivant a tranhow releculate """Cal      t:
  ) -> floa  t: str
  contex       ion: str,
  translat
             self,nce(
  ntext_relevacoate_calcul def _  async
    
  alternativesturn           re {e}")
  tives:ing alternacorrror s(f"Eerrorr.  logge  e:
        s ption acecept Ex ex   
                rue)
erse=T revonfidence,x: x.cy=lambda s, ketiveernasorted(altreturn          core
   e senc confid # Sort by         
            .0)
  (score, 1nce = mine.confideternativ    al      ore
      inal scence with f confid   # Update         
               oost)
     ference_b + pre(1*= e    scor            )
                        
 eferencesprive, user_rnat       alte                 erences(
pref_user_pply._a = selfrence_boost       prefe     :
        ncesser_prefere      if u     ts
     stmene adjurencfeer pre        # Us     
          )
         weight"]ext_"contweights[main_* do_relevance + contextscore *= (1                     )
               xt
     .text, conteernativelt         a        (
       levancentext_reate_co._calculawait self= _relevance text con             
      ":== "aiurce ternative.so alt andexif cont       
         e boostanclevContext re     #               
       ons
      slatianst human tr.2  # Boocore *= 1   s               uman":
  rce == "hrnative.soulte  elif a             )
 "]ghtemory_weiweights["mmain_*= (1 + do      score          
      "memory":ce ==ve.sourrnatiteelif al        
        weight"])minology_"tereights[domain_w(1 + *=       score    
           gy":"terminolosource == rnative.    if alte            ic weights
main-specif# Apply do                          

      ceidenive.conf= alternat  score            e score
   nfidenc Base co    #            tives:
rnatetive in alternaor al        f 
            ])
   ICALTECHNDomain.lationansTrweights[n_.domaielft(domain, sn_weights.gedomaif.eights = selomain_w          d try:
  ""
       ives."aterntion alt transla and rank""Score
        "tive]:ionAlternaranslatist[T
    ) -> Ltr, Any]]nal[Dict[sOptionces: reser_prefe,
        ustr]nal[Optioext:   cont,
      ionDomainanslat Tr    domain:,
    _text: strource        se],
nAlternativ[Translatioves: Listernatialt         self,
 s(
      ive_alternatf _scorenc desy    a
    
  return []      )
    e}" {es: alternativerminology getting t"Error(f.errorger      log:
      ption as except Exce   e    
     
        return []          
        
           )]          }
                   used)
  logy_n(termino": leterms_coun"t                d,
        _useinologyed": termnology_appli  "termi                    ata={
  metad               main,
     doain=         dom           ,
minology"er="t source                 .85,
  nce=0ide    conf               ed_text,
 text=modifi               ve(
     ternatiionAl[Translatn    retur           
  used:y_ogrminol    if te      
    
           })          n
         maintry.do emain":     "do              tion,
     ntry.translalation": e    "trans                 erm,
   : entry.t"term"               
         ed.append({ogy_usnol       termi           n)
  .translatiom, entryry.tere(entac_text.repldified = moified_text  mod              er():
    xt.low_te() in sourceterm.lower entry.      if      
    ntries:gy_elonory in termint      for e
                 ]
 _used = [gyerminolo       t  
   ce_textour= sied_text dif          mot
  urce texology to soly termin    # App      
             .all()
         )  
      )           == True
 ed ssary.approvologyGlo  Termin            
      uage,angget_l == tarnguaget_lay.targeossarinologyGl     Term              ,
 e_languagesourc= age =e_languourcyGlossary.sminolog     Ter             and_(
                ilter(
  ossary).flogyGlnoermidb.query(Ties = trology_enrmin        te
    rminologylevant te # Find re           try:
"
        ""lternatives.on aranslatiy-based tologinerm"Get t   ""   ve]:
  atitionAlternransla -> List[T   )ssion
 Se   db: ain,
     slationDomn: Tranmai do,
       uage: strt_lang     targe,
   trnguage: srce_la       souxt: str,
 te     source_  self,
        ves(
 alternatinology_et_termiasync def _g    
    urn []
ret    )
        " {e}es:n alternativng huma"Error gettier.error(f      loggs e:
      n aioptpt Exce      exce   
           e)[:2]
rse=Trurevece,  x.confidenambda x:tives, key=lalternan sorted(   retur                 
  ))
                       }
                       alue
s.vtatuation_sanslslation.tr: tranw_status"     "revie                   score,
    quality_ation.: transly_score"lit   "qua                        ,
 tor_idtion.transla": translaor_idslat     "tran                      data={
     meta               ICAL,
     .TECHNlationDomainTransmain=do                        uman",
 source="h               
         0.9,score orquality_anslation.e=trconfidenc                      ,
  .contentranslation     text=t                  ative(
 lationAlternappend(Transes. alternativ                   4:
 > 0.n.content)latioxt, transty(source_teariilmple_simand self._sicontent translation. if             ons:
   latin_trans humaon inlatir trans  fo
           = []rnatives alte            
   )
        mit(50).all(    ).li             )
  
          0.8e >=corquality_sion.entTranslat        Cont    
        SHED,LIStatus.PUBnslationtus == Trastatranslation_ion.tentTranslaton           C       ,
  NUALethod.MAranslationMthod == Tslation_metranon.slati ContentTran                uage,
   arget_lang te ==language_codion.Translatontent          C        and_(
              lter(
    lation).fitentTrans.query(Condb = onsranslatiuman_t          h  ase
he databons in tan translatiook for hum  # L             try:
 "
    "".vesternatin alranslatio tewed human-revi   """Get     tive]:
nAlternaslatio List[Tran ->   )Session
     db:   : str,
  uageet_lang        targ
uage: str,source_lang     r,
   ext: strce_t      sou   self,
es(
       lternativ_human_anc def _getsy
    a    ]"
t}rce_tex {souion: translate}nammodel_[{"  return f   
      t}]"
     texurce_ {soslation of:an} trmodel_nameeturn f"[{          r          else:
       یس"
 ابrn "ڈیٹ retu          er():
     t.lowrce_tex" in souasedatablif "           eلگورتھم"
 "ا    return       
      ower():_text.lource s" inhm"algoritif     el   گ"
     ین لرننn "مش   retur       
      xt.lower():ource_terning" in seachine l   if "ma  u
        Urdulation forimle smp # Si
            == "ur":get_languagear       if t 
 