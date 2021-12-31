import pandas as pd
import re
import tensorflow as tf
import tensorflow_datasets as tfds
import urllib.request

from conversation import transformer, ConversationSchedule
from helpers import get_model_path, get_dataset_path


if __name__ == '__main__':
    # ailab dataset, 26816
    dataset_path = get_dataset_path('justfortest')
    train_data = pd.read_csv(dataset_path, names=['Q', 'A'])

    # wiki 일상 dataset, 11823
    dataset_path = get_dataset_path('wiki/ChatBotData.csv', require_directory=True)
    urllib.request.urlretrieve('https://raw.githubusercontent.com/songys/Chatbot_data/master/ChatbotData.csv', filename=dataset_path)
    train_data = pd.concat([train_data, pd.read_csv(dataset_path)])

    # w 대본 dataset, 5702
    dataset_path = get_dataset_path('script_W/conversations/QAconversation.csv')
    train_data = pd.concat([train_data, pd.read_csv(dataset_path, names=['Q', 'A'])[:5702]])

    # 건축학개론 대본 datset, 987
    dataset_path = get_dataset_path('script_architecture101/dataset.csv')
    train_data = pd.concat([train_data, pd.read_csv(dataset_path, names=['Q', 'A'])])

    # 모두의 말뭉치 일상 대화 dataset, 74674
    dataset_path = get_dataset_path('SDRW/conversation')
    train_data = pd.concat([train_data, pd.read_csv(dataset_path, names=['Q', 'A'])])

    # Cleaning
    train_data = train_data.dropna(axis=0)

    questions = []
    for sentence in train_data['Q']:
        # 구두점에 대해서 띄어쓰기
        # ex) 12시 땡! -> 12시 땡 !
        sentence = re.sub(r"([?.!,])", r" \1 ", sentence)
        sentence = sentence.strip()
        questions.append(sentence)

    answers = []
    for sentence in train_data['A']:
        # 구두점에 대해서 띄어쓰기
        # ex) 12시 땡! -> 12시 땡 !
        sentence = re.sub(r"([?.!,])", r" \1 ", sentence)
        sentence = sentence.strip()
        answers.append(sentence)

    print(questions[:5])
    print(answers[:5])

    tokenizer = tfds.deprecated.text.SubwordTextEncoder.build_from_corpus(
        questions + answers,
        target_vocab_size=2 ** 13)

    tokenizer_path = get_model_path('conversation/tokenizer', require_directory=True)
    tokenizer.save_to_file(tokenizer_path)

    # 시작 토큰과 종료 토큰에 대한 정수 부여
    START_TOKEN, END_TOKEN = [tokenizer.vocab_size], [tokenizer.vocab_size + 1]

    # 시작 토큰과 종료 토큰을 고려하여 단어 집합의 크기 + 2
    VOCAB_SIZE = tokenizer.vocab_size + 2

    print(f'START_TOKEN={START_TOKEN}')
    print(f'END_TOKEN={END_TOKEN}')
    print(f'VOCAB_SIZE={VOCAB_SIZE}')

    # 최대 길이를 40으로 정의
    MAX_LENGTH = 40


    # 토큰화 / 정수 인코딩 / 시작 토큰과 종료 토큰 추가 / 패딩
    def tokenize_and_filter(inputs, outputs):
        tokenized_inputs, tokenized_outputs = [], []

        for (sentence1, sentence2) in zip(inputs, outputs):
            # encode(토큰화 + 정수 인코딩), 시작 토큰과 종료 토큰 추가
            sentence1 = START_TOKEN + tokenizer.encode(sentence1) + END_TOKEN
            sentence2 = START_TOKEN + tokenizer.encode(sentence2) + END_TOKEN

            tokenized_inputs.append(sentence1)
            tokenized_outputs.append(sentence2)

        # 패딩
        tokenized_inputs = tf.keras.preprocessing.sequence.pad_sequences(
            tokenized_inputs, maxlen=MAX_LENGTH, padding='post')
        tokenized_outputs = tf.keras.preprocessing.sequence.pad_sequences(
            tokenized_outputs, maxlen=MAX_LENGTH, padding='post')

        return tokenized_inputs, tokenized_outputs


    questions, answers = tokenize_and_filter(questions, answers)

    print(f'Questions shape={questions.shape}')
    print(f'Answers shape={answers.shape}')

    # TensorFlow Dataset을 이용하여 shuffle을 수행하되, 배치 크기로 데이터를 묶는다.
    # 또한 이 과정에서 teacher forcing을 사용하기 위해서 디코더의 입력과 실제값 시퀀스를 구성한다.
    BATCH_SIZE = 64
    BUFFER_SIZE = 20000

    # 디코더의 sequence에서는 START_TOKEN을 제거해야 한다.
    dataset = tf.data.Dataset.from_tensor_slices((
        {
            'inputs': questions,
            'dec_inputs': answers[:, :-1]  # 마지막 패딩 토큰이 제거된다.
        },
        {
            'outputs': answers[:, 1:]  # 시작 토큰이 제거된다.
        }
    ))

    dataset = dataset.cache()
    dataset = dataset.shuffle(BUFFER_SIZE)
    dataset = dataset.batch(BATCH_SIZE)
    dataset = dataset.prefetch(tf.data.experimental.AUTOTUNE)

    tf.keras.backend.clear_session()

    # Hyper-parameters
    D_MODEL = 256
    NUM_LAYERS = 2
    NUM_HEADS = 8
    DFF = 512
    DROPOUT = 0.1

    model = transformer(
        vocab_size=VOCAB_SIZE,
        num_layers=NUM_LAYERS,
        dff=DFF,
        d_model=D_MODEL,
        num_heads=NUM_HEADS,
        dropout=DROPOUT)

    learning_rate = ConversationSchedule(D_MODEL)

    optimizer = tf.keras.optimizers.Adam(
        learning_rate,
        beta_1=0.9,
        beta_2=0.98,
        epsilon=1e-9)


    def loss_function(y_true, y_pred):
        y_true = tf.reshape(y_true, shape=(-1, MAX_LENGTH - 1))

        loss = tf.keras.losses.SparseCategoricalCrossentropy(
            from_logits=True,
            reduction='none')

        loss = loss(y_true, y_pred)
        mask = tf.cast(tf.not_equal(y_true, 0), tf.float32)
        loss = tf.multiply(loss, mask)

        return tf.reduce_mean(loss)


    def accuracy(y_true, y_pred):
        y_true = tf.reshape(y_true, shape=(-1, MAX_LENGTH - 1))
        return tf.keras.metrics.sparse_categorical_accuracy(y_true, y_pred)


    model.compile(
        optimizer=optimizer,
        loss=loss_function,
        metrics=[accuracy])

    EPOCHS = 50
    model.fit(dataset, epochs=EPOCHS)

    # 훈련된 모델 저장
    model_path = get_model_path('conversation/saved_model', require_directory=True)
    model.save(model_path, include_optimizer=False)
